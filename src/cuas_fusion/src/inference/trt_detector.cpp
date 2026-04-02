// trt_detector.cpp
// TensorRT YOLOv8 inference: engine deserialization, pre-process (resize +
// normalize + HWC→CHW), async GPU inference, post-process (confidence filter
// + NMS), box rescaling to original frame dimensions.

#include "cuas_fusion/inference/trt_detector.hpp"
#include "cuas_fusion/common/clock.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <fstream>
#include <vector>
#include <cstring>

namespace cuas {

// ---------------------------------------------------------------------------
// init — deserialize engine, create context, pre-allocate all buffers
// ---------------------------------------------------------------------------
bool TrtDetector::init(const std::string& engine_path)
{
    if (initialized_) {
        fprintf(stderr, "[TrtDetector] already initialized\n");
        return false;
    }

    // ---- read serialized engine from disk ----
    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        fprintf(stderr, "[TrtDetector] cannot open engine: %s\n", engine_path.c_str());
        return false;
    }
    const size_t file_size = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(file_size);
    if (!file.read(engine_data.data(), static_cast<std::streamsize>(file_size))) {
        fprintf(stderr, "[TrtDetector] failed to read engine file\n");
        return false;
    }
    file.close();

    // ---- create runtime + deserialize ----
    nvinfer1::IRuntime* raw_runtime = nvinfer1::createInferRuntime(logger_);
    if (!raw_runtime) {
        fprintf(stderr, "[TrtDetector] createInferRuntime failed\n");
        return false;
    }
    runtime_ = RuntimePtr(raw_runtime, [](nvinfer1::IRuntime* p) { delete p; });

    nvinfer1::ICudaEngine* raw_engine =
        runtime_->deserializeCudaEngine(engine_data.data(), file_size);
    if (!raw_engine) {
        fprintf(stderr, "[TrtDetector] deserializeCudaEngine failed\n");
        return false;
    }
    engine_ = EnginePtr(raw_engine, [](nvinfer1::ICudaEngine* p) { delete p; });

    // ---- discover I/O tensor names ----
    const int32_t nb_io = engine_->getNbIOTensors();
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_name_ = name;
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            output_name_ = name;
        }
    }
    if (!input_name_ || !output_name_) {
        fprintf(stderr, "[TrtDetector] failed to discover I/O tensor names\n");
        return false;
    }

    // ---- create execution context ----
    nvinfer1::IExecutionContext* raw_ctx = engine_->createExecutionContext();
    if (!raw_ctx) {
        fprintf(stderr, "[TrtDetector] createExecutionContext failed\n");
        return false;
    }
    context_ = ContextPtr(raw_ctx, [](nvinfer1::IExecutionContext* p) { delete p; });

    // ---- compute buffer sizes ----
    // Input: 1 x 3 x 640 x 640 float32
    input_bytes_ = static_cast<size_t>(1) * INFERENCE_INPUT_C
                 * INFERENCE_INPUT_H * INFERENCE_INPUT_W * sizeof(float);
    // Output: 1 x (4 + num_classes) x 8400 float32
    constexpr int rows = 4 + INFERENCE_NUM_CLASSES;  // 84
    output_bytes_ = static_cast<size_t>(1) * rows * INFERENCE_NUM_ANCHORS * sizeof(float);

    // ---- create CUDA stream ----
    cudaStream_t raw_stream = nullptr;
    if (cudaStreamCreate(&raw_stream) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] cudaStreamCreate failed\n");
        return false;
    }
    stream_ = CudaStreamPtr(raw_stream);

    // ---- allocate pinned host buffers ----
    void* h_in = nullptr;
    void* h_out = nullptr;
    if (cudaMallocHost(&h_in, input_bytes_) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] cudaMallocHost input failed\n");
        return false;
    }
    input_host_ = HostBufPtr(h_in);

    if (cudaMallocHost(&h_out, output_bytes_) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] cudaMallocHost output failed\n");
        return false;
    }
    output_host_ = HostBufPtr(h_out);

    // ---- allocate device buffers ----
    void* d_in = nullptr;
    void* d_out = nullptr;
    if (cudaMalloc(&d_in, input_bytes_) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] cudaMalloc input failed\n");
        return false;
    }
    input_dev_ = DeviceBufPtr(d_in);

    if (cudaMalloc(&d_out, output_bytes_) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] cudaMalloc output failed\n");
        return false;
    }
    output_dev_ = DeviceBufPtr(d_out);

    // ---- bind tensor addresses to context ----
    if (!context_->setTensorAddress(input_name_, input_dev_.get())) {
        fprintf(stderr, "[TrtDetector] setTensorAddress(%s) failed\n", input_name_);
        return false;
    }
    if (!context_->setTensorAddress(output_name_, output_dev_.get())) {
        fprintf(stderr, "[TrtDetector] setTensorAddress(%s) failed\n", output_name_);
        return false;
    }

    initialized_ = true;
    fprintf(stderr, "[TrtDetector] initialized: input=%s output=%s  in=%.1f KB  out=%.1f KB\n",
            input_name_, output_name_,
            static_cast<double>(input_bytes_) / 1024.0,
            static_cast<double>(output_bytes_) / 1024.0);
    return true;
}

// ---------------------------------------------------------------------------
// preprocess — resize, normalize /255, HWC→CHW into pinned host buffer
// ---------------------------------------------------------------------------
bool TrtDetector::preprocess(const cv::Mat& bgr_frame)
{
    if (bgr_frame.empty()) {
        fprintf(stderr, "[TrtDetector] preprocess: empty frame\n");
        return false;
    }

    // Resize to network input dimensions
    cv::Mat resized;
    cv::resize(bgr_frame, resized,
               cv::Size(INFERENCE_INPUT_W, INFERENCE_INPUT_H),
               0.0, 0.0, cv::INTER_LINEAR);

    // Convert to float32 and normalize by 255
    cv::Mat float_img;
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // One-time range check — catch non-8-bit input from camera ISP
    if (!range_warned_) {
        double min_val = 0.0, max_val = 0.0;
        cv::minMaxLoc(float_img.reshape(1), &min_val, &max_val);
        if (max_val > 1.0) {
            fprintf(stderr, "[TrtDetector] WARNING: normalized pixel max=%.3f > 1.0 "
                    "— input was not 8-bit (type=%d). Check camera ISP pipeline.\n",
                    max_val, bgr_frame.type());
            range_warned_ = true;
        }
    }

    // HWC → CHW plane split into pinned buffer
    // OpenCV BGR split: channels[0]=B, [1]=G, [2]=R
    // YOLOv8 expects RGB: plane 0=R, plane 1=G, plane 2=B
    float* dst = static_cast<float*>(input_host_.get());
    const size_t plane_size = static_cast<size_t>(INFERENCE_INPUT_H) * INFERENCE_INPUT_W;

    cv::Mat channels[3];
    cv::split(float_img, channels);
    std::memcpy(dst,                      channels[2].data, plane_size * sizeof(float));  // R
    std::memcpy(dst + plane_size,         channels[1].data, plane_size * sizeof(float));  // G
    std::memcpy(dst + 2 * plane_size,     channels[0].data, plane_size * sizeof(float));  // B

    return true;
}

// ---------------------------------------------------------------------------
// postprocess — decode [1,84,8400], confidence filter, NMS, rescale boxes
// ---------------------------------------------------------------------------
bool TrtDetector::postprocess(std::vector<BoundingBox>& detections_out,
                              int orig_w, int orig_h, int64_t timestamp_ns)
{
    detections_out.clear();

    const float* raw = static_cast<const float*>(output_host_.get());
    // Layout: raw[row * 8400 + anchor], row 0-3 = xywh, rows 4-83 = class scores

    const float scale_x = static_cast<float>(orig_w) / static_cast<float>(INFERENCE_INPUT_W);
    const float scale_y = static_cast<float>(orig_h) / static_cast<float>(INFERENCE_INPUT_H);

    std::vector<BoundingBox> candidates;
    candidates.reserve(512);

    for (int a = 0; a < INFERENCE_NUM_ANCHORS; ++a) {
        // Find max class score for this anchor
        float max_score = 0.0f;
        int   max_cls   = 0;
        for (int c = 0; c < INFERENCE_NUM_CLASSES; ++c) {
            const float s = raw[static_cast<size_t>(4 + c) * INFERENCE_NUM_ANCHORS + a];
            if (s > max_score) {
                max_score = s;
                max_cls   = c;
            }
        }

        if (max_score < INFERENCE_CONF_THRESH) {
            continue;
        }

        // Decode center-format xywh in network-input pixel space
        const float cx = raw[static_cast<size_t>(0) * INFERENCE_NUM_ANCHORS + a];
        const float cy = raw[static_cast<size_t>(1) * INFERENCE_NUM_ANCHORS + a];
        const float bw = raw[static_cast<size_t>(2) * INFERENCE_NUM_ANCHORS + a];
        const float bh = raw[static_cast<size_t>(3) * INFERENCE_NUM_ANCHORS + a];

        // Convert to top-left origin and scale to original frame coords
        BoundingBox det{};
        det.x            = (cx - bw * 0.5f) * scale_x;
        det.y            = (cy - bh * 0.5f) * scale_y;
        det.w            = bw * scale_x;
        det.h            = bh * scale_y;
        det.confidence   = max_score;
        det.class_id     = max_cls;
        det.timestamp_ns = timestamp_ns;

        candidates.push_back(det);
    }

    // NMS
    nms(candidates, INFERENCE_NMS_THRESH);

    // Cap to max detections
    if (candidates.size() > static_cast<size_t>(INFERENCE_MAX_DET)) {
        candidates.resize(INFERENCE_MAX_DET);
    }

    detections_out = std::move(candidates);
    return true;
}

// ---------------------------------------------------------------------------
// IoU — intersection over union between two top-left xywh boxes
// ---------------------------------------------------------------------------
float TrtDetector::iou(const BoundingBox& a, const BoundingBox& b)
{
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.w, b.x + b.w);
    const float y2 = std::min(a.y + a.h, b.y + b.h);

    const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float area_a = a.w * a.h;
    const float area_b = b.w * b.h;
    const float union_area = area_a + area_b - inter;

    return (union_area > 0.0f) ? (inter / union_area) : 0.0f;
}

// ---------------------------------------------------------------------------
// NMS — greedy per-class non-maximum suppression, in-place
// ---------------------------------------------------------------------------
void TrtDetector::nms(std::vector<BoundingBox>& dets, float thresh)
{
    std::sort(dets.begin(), dets.end(),
              [](const BoundingBox& a, const BoundingBox& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<bool> suppressed(dets.size(), false);
    std::vector<BoundingBox> kept;
    kept.reserve(dets.size());

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        kept.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (suppressed[j]) continue;
            if (dets[i].class_id == dets[j].class_id &&
                iou(dets[i], dets[j]) > thresh) {
                suppressed[j] = true;
            }
        }
    }

    dets = std::move(kept);
}

// ---------------------------------------------------------------------------
// infer — full pipeline: preprocess → H2D → execute → D2H → postprocess
// ---------------------------------------------------------------------------
bool TrtDetector::infer(const cv::Mat& bgr_frame,
                        std::vector<BoundingBox>& detections_out)
{
    detections_out.clear();

    if (!initialized_) {
        fprintf(stderr, "[TrtDetector] infer called before init\n");
        return false;
    }

    const int64_t timestamp_ns = now_ns();
    const int orig_w = bgr_frame.cols;
    const int orig_h = bgr_frame.rows;

    // 1. preprocess (resize, /255, HWC→CHW)
    if (!preprocess(bgr_frame)) {
        return false;
    }

    // 2. H2D async
    if (cudaMemcpyAsync(input_dev_.get(), input_host_.get(), input_bytes_,
                        cudaMemcpyHostToDevice, stream_.get()) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] H2D memcpy failed\n");
        return false;
    }

    // 3. execute inference
    if (!context_->enqueueV3(stream_.get())) {
        fprintf(stderr, "[TrtDetector] enqueueV3 failed\n");
        return false;
    }

    // 4. D2H async
    if (cudaMemcpyAsync(output_host_.get(), output_dev_.get(), output_bytes_,
                        cudaMemcpyDeviceToHost, stream_.get()) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] D2H memcpy failed\n");
        return false;
    }

    // 5. synchronize
    if (cudaStreamSynchronize(stream_.get()) != cudaSuccess) {
        fprintf(stderr, "[TrtDetector] stream sync failed\n");
        return false;
    }

    // 6. postprocess (decode + NMS)
    return postprocess(detections_out, orig_w, orig_h, timestamp_ns);
}

// ---------------------------------------------------------------------------
// shutdown — release all resources in reverse creation order
// ---------------------------------------------------------------------------
void TrtDetector::shutdown()
{
    if (!initialized_) return;
    initialized_ = false;

    if (stream_) {
        cudaStreamSynchronize(stream_.get());
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

// ---------------------------------------------------------------------------
bool TrtDetector::is_initialized() const
{
    return initialized_;
}

} // namespace cuas
