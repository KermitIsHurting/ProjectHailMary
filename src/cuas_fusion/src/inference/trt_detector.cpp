// @file trt_detector.cpp
// @brief TensorRT engine loader and YOLO preprocessing/postprocessing pipeline.
#include "cuas_fusion/inference/trt_detector.hpp"
#include "cuas_fusion/common/clock.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace cuas {

bool TrtDetector::init(const std::string& engine_path)
{
    if (initialized_) {
        return false;
    }

    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }
    const std::size_t file_size = static_cast<std::size_t>(file.tellg());
    file.seekg(0, std::ios::beg);
    // WHY: TensorRT deserializeEngine requires contiguous char buffer of unknown
    // size at load time; this is a one-shot init allocation (DEV-004).
    std::vector<char> engine_data(file_size);
    if (!file.read(engine_data.data(), static_cast<std::streamsize>(file_size))) {
        return false;
    }
    file.close();

    nvinfer1::IRuntime* raw_runtime = nvinfer1::createInferRuntime(logger_);
    if (raw_runtime == nullptr) {
        return false;
    }
    runtime_ = RuntimePtr(raw_runtime);

    nvinfer1::ICudaEngine* raw_engine =
        runtime_->deserializeCudaEngine(engine_data.data(), file_size);
    if (raw_engine == nullptr) {
        return false;
    }
    engine_ = EnginePtr(raw_engine);

    // Exactly one input and one output (A2.8): a multi-tensor engine used
    // to silently keep whichever tensor the loop saw last.
    int32_t n_inputs  = 0;
    int32_t n_outputs = 0;
    const int32_t nb_io = engine_->getNbIOTensors();
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = engine_->getIOTensorName(i);
        const nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            ++n_inputs;
            input_name_ = name;
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            ++n_outputs;
            output_name_ = name;
        } else {
        }
    }
    if ((n_inputs != 1) || (n_outputs != 1) ||
        (input_name_ == nullptr) || (output_name_ == nullptr)) {
        std::fprintf(stderr,
            "TrtDetector: engine has %d inputs / %d outputs, expected 1/1\n",
            n_inputs, n_outputs);
        return false;
    }

    // Shape validation against the compile-time buffers (A2.8): a different
    // YOLO variant engine would make enqueueV3 write past output_dev_ — a
    // device-side overflow with no diagnostic.
    const auto dims_match = [](const nvinfer1::Dims& d,
                               const std::array<int64_t, 4>& expect,
                               int32_t nb) -> bool {
        if (d.nbDims != nb) {
            return false;
        }
        for (int32_t k = 0; k < nb; ++k) {
            if (static_cast<int64_t>(d.d[k]) != expect[static_cast<std::size_t>(k)]) {
                return false;
            }
        }
        return true;
    };
    const nvinfer1::Dims in_dims  = engine_->getTensorShape(input_name_);
    const nvinfer1::Dims out_dims = engine_->getTensorShape(output_name_);
    const std::array<int64_t, 4> expect_in{
        1, INFERENCE_INPUT_C, INFERENCE_INPUT_H, INFERENCE_INPUT_W};
    const std::array<int64_t, 4> expect_out{
        1, 4 + INFERENCE_NUM_CLASSES, INFERENCE_NUM_ANCHORS, 0};
    if (!dims_match(in_dims, expect_in, 4) ||
        !dims_match(out_dims, expect_out, 3)) {
        std::fprintf(stderr,
            "TrtDetector: engine tensor shapes do not match the compiled "
            "YOLOv8 buffers (expected input 1x%dx%dx%d, output 1x%dx%d)\n",
            INFERENCE_INPUT_C, INFERENCE_INPUT_H, INFERENCE_INPUT_W,
            4 + INFERENCE_NUM_CLASSES, INFERENCE_NUM_ANCHORS);
        return false;
    }

    nvinfer1::IExecutionContext* raw_ctx = engine_->createExecutionContext();
    if (raw_ctx == nullptr) {
        return false;
    }
    context_ = ContextPtr(raw_ctx);

    input_bytes_  = static_cast<std::size_t>(1) * INFERENCE_INPUT_C
                 * INFERENCE_INPUT_H * INFERENCE_INPUT_W * sizeof(float32_t);
    constexpr int32_t rows = 4 + INFERENCE_NUM_CLASSES;
    output_bytes_ = static_cast<std::size_t>(1) * rows * INFERENCE_NUM_ANCHORS * sizeof(float32_t);

    cudaStream_t raw_stream = nullptr;
    if (cudaStreamCreate(&raw_stream) != cudaSuccess) {
        return false;
    }
    stream_ = CudaStreamPtr(raw_stream);

    void* h_in = nullptr;
    void* h_out = nullptr;
    if (cudaMallocHost(&h_in, input_bytes_) != cudaSuccess) {
        return false;
    }
    input_host_ = HostBufPtr(h_in);

    if (cudaMallocHost(&h_out, output_bytes_) != cudaSuccess) {
        return false;
    }
    output_host_ = HostBufPtr(h_out);

    void* d_in = nullptr;
    void* d_out = nullptr;
    if (cudaMalloc(&d_in, input_bytes_) != cudaSuccess) {
        return false;
    }
    input_dev_ = DeviceBufPtr(d_in);

    if (cudaMalloc(&d_out, output_bytes_) != cudaSuccess) {
        return false;
    }
    output_dev_ = DeviceBufPtr(d_out);

    if (!context_->setTensorAddress(input_name_, input_dev_.get())) {
        return false;
    }
    if (!context_->setTensorAddress(output_name_, output_dev_.get())) {
        return false;
    }

    preproc_resized_ = cv::Mat(INFERENCE_INPUT_H, INFERENCE_INPUT_W, CV_8UC3);
    preproc_float_   = cv::Mat(INFERENCE_INPUT_H, INFERENCE_INPUT_W, CV_32FC3);
    for (auto& channel : preproc_channels_) {
        channel = cv::Mat(INFERENCE_INPUT_H, INFERENCE_INPUT_W, CV_32FC1);
    }

    initialized_ = true;
    return true;
}

bool TrtDetector::preprocess(const cv::Mat& bgr_frame)
{
    if (bgr_frame.empty()) {
        return false;
    }

    cv::Mat& resized = preproc_resized_;
    cv::resize(bgr_frame, resized,
               cv::Size(INFERENCE_INPUT_W, INFERENCE_INPUT_H),
               0.0, 0.0, cv::INTER_LINEAR);

    cv::Mat& float_img = preproc_float_;
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // One-time sanity check — values above 1.0 indicate the ISP isn't 8-bit
    if (!range_warned_) {
        float64_t min_val = 0.0;
        float64_t max_val = 0.0;
        cv::minMaxLoc(float_img.reshape(1), &min_val, &max_val);
        if (max_val > 1.0) {
            range_warned_ = true;
        }
    }

    // YOLOv8 expects RGB but OpenCV provides BGR, so swap channels on copy
    float32_t* dst = static_cast<float32_t*>(input_host_.get());
    const std::size_t plane_size =
        static_cast<std::size_t>(INFERENCE_INPUT_H) * INFERENCE_INPUT_W;

    std::array<cv::Mat, 3>& channels = preproc_channels_;
    cv::split(float_img, channels.data());
    std::memcpy(dst,                        channels[2].data, plane_size * sizeof(float32_t));
    std::memcpy(dst +     plane_size,       channels[1].data, plane_size * sizeof(float32_t));
    std::memcpy(dst + 2 * plane_size,       channels[0].data, plane_size * sizeof(float32_t));

    return true;
}

bool TrtDetector::postprocess(FixedVector<BoundingBox, 128U>& detections_out,
                              int32_t orig_w, int32_t orig_h, int64_t timestamp_ns)
{
    detections_out.clear();

    const float32_t* raw = static_cast<const float32_t*>(output_host_.get());

    const float32_t scale_x = static_cast<float32_t>(orig_w) / static_cast<float32_t>(INFERENCE_INPUT_W);
    const float32_t scale_y = static_cast<float32_t>(orig_h) / static_cast<float32_t>(INFERENCE_INPUT_H);

    FixedVector<BoundingBox, 128U> candidates;

    for (int32_t a = 0; a < INFERENCE_NUM_ANCHORS; ++a) {
        float32_t max_score = 0.0F;
        int32_t   max_cls   = 0;
        for (int32_t c = 0; c < INFERENCE_NUM_CLASSES; ++c) {
            const float32_t s = raw[static_cast<std::size_t>(4 + c) * INFERENCE_NUM_ANCHORS + a];
            if (s > max_score) {
                max_score = s;
                max_cls   = c;
            }
        }

        if (max_score < INFERENCE_CONF_THRESH) {
            continue;
        }

        const float32_t cx = raw[static_cast<std::size_t>(0) * INFERENCE_NUM_ANCHORS + a];
        const float32_t cy = raw[static_cast<std::size_t>(1) * INFERENCE_NUM_ANCHORS + a];
        const float32_t bw = raw[static_cast<std::size_t>(2) * INFERENCE_NUM_ANCHORS + a];
        const float32_t bh = raw[static_cast<std::size_t>(3) * INFERENCE_NUM_ANCHORS + a];

        BoundingBox det{};
        det.x            = (cx - bw * 0.5F) * scale_x;
        det.y            = (cy - bh * 0.5F) * scale_y;
        det.w            = bw * scale_x;
        det.h            = bh * scale_y;
        det.confidence   = max_score;
        det.class_id     = max_cls;
        det.timestamp_ns = timestamp_ns;

        if (!candidates.push_back(det)) {
            break;
        }
    }

    nms(candidates, INFERENCE_NMS_THRESH);

    if (candidates.size() > static_cast<uint32_t>(INFERENCE_MAX_DET)) {
        // Shrink cannot fail: INFERENCE_MAX_DET <= capacity by construction.
        (void)candidates.resize(static_cast<uint32_t>(INFERENCE_MAX_DET));
    }

    detections_out = candidates;
    return true;
}

float32_t TrtDetector::iou(const BoundingBox& a, const BoundingBox& b)
{
    const float32_t x1 = std::max(a.x, b.x);
    const float32_t y1 = std::max(a.y, b.y);
    const float32_t x2 = std::min(a.x + a.w, b.x + b.w);
    const float32_t y2 = std::min(a.y + a.h, b.y + b.h);

    const float32_t inter = std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
    const float32_t area_a = a.w * a.h;
    const float32_t area_b = b.w * b.h;
    const float32_t union_area = area_a + area_b - inter;

    if (union_area > 0.0F) {
        return inter / union_area;
    }
    return 0.0F;
}

void TrtDetector::nms(FixedVector<BoundingBox, 128U>& dets, float32_t thresh)
{
    std::sort(dets.begin(), dets.end(),
              [](const BoundingBox& a, const BoundingBox& b) {
                  return a.confidence > b.confidence;
              });

    FixedVector<uint8_t, 128U> suppressed;
    for (uint32_t i = 0U; i < dets.size(); ++i) {
        (void)suppressed.push_back(0U);
    }

    FixedVector<BoundingBox, 128U> kept;

    for (uint32_t i = 0U; i < dets.size(); ++i) {
        if (suppressed[i] != 0U) {
            continue;
        }
        (void)kept.push_back(dets[i]);
        for (uint32_t j = i + 1U; j < dets.size(); ++j) {
            if (suppressed[j] != 0U) {
                continue;
            }
            if (dets[i].class_id == dets[j].class_id &&
                iou(dets[i], dets[j]) > thresh) {
                suppressed[j] = 1U;
            }
        }
    }

    dets = kept;
}

bool TrtDetector::infer(const cv::Mat& bgr_frame,
                        FixedVector<BoundingBox, 128U>& detections_out)
{
    detections_out.clear();

    if (!initialized_) {
        return false;
    }

    const int64_t timestamp_ns = now_ns();
    const int32_t orig_w = bgr_frame.cols;
    const int32_t orig_h = bgr_frame.rows;

    if (!preprocess(bgr_frame)) {
        return false;
    }

    if (cudaMemcpyAsync(input_dev_.get(), input_host_.get(), input_bytes_,
                        cudaMemcpyHostToDevice, stream_.get()) != cudaSuccess) {
        return false;
    }

    if (!context_->enqueueV3(stream_.get())) {
        return false;
    }

    if (cudaMemcpyAsync(output_host_.get(), output_dev_.get(), output_bytes_,
                        cudaMemcpyDeviceToHost, stream_.get()) != cudaSuccess) {
        return false;
    }

    if (cudaStreamSynchronize(stream_.get()) != cudaSuccess) {
        return false;
    }

    return postprocess(detections_out, orig_w, orig_h, timestamp_ns);
}

void TrtDetector::shutdown()
{
    if (!initialized_) {
        return;
    }
    initialized_ = false;

    if (stream_) {
        (void)cudaStreamSynchronize(stream_.get());
    }

    output_dev_.reset();
    input_dev_.reset();
    output_host_.reset();
    input_host_.reset();
    stream_.reset();
    context_.reset();
    engine_.reset();
    runtime_.reset();

    range_warned_ = false;
    input_name_   = nullptr;
    output_name_  = nullptr;
}

bool TrtDetector::is_initialized() const
{
    return initialized_;
}

} // namespace cuas
