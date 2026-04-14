// @file trt_detector.hpp
// @brief TensorRT YOLO object detector wrapper.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace cuas {

struct CudaStreamDeleter {
    void operator()(cudaStream_t s) const {
        if (s != nullptr) {
            (void)cudaStreamDestroy(s);
        }
    }
};

struct CudaFreeDeleter {
    void operator()(void* p) const {
        if (p != nullptr) {
            (void)cudaFree(p);
        }
    }
};

struct CudaFreeHostDeleter {
    void operator()(void* p) const {
        if (p != nullptr) {
            (void)cudaFreeHost(p);
        }
    }
};

using RuntimePtr    = std::unique_ptr<nvinfer1::IRuntime,  void(*)(nvinfer1::IRuntime*)>;
using EnginePtr     = std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)>;
using ContextPtr    = std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)>;
using CudaStreamPtr = std::unique_ptr<std::remove_pointer<cudaStream_t>::type, CudaStreamDeleter>;
using DeviceBufPtr  = std::unique_ptr<void, CudaFreeDeleter>;
using HostBufPtr    = std::unique_ptr<void, CudaFreeHostDeleter>;

class TrtDetector {
public:
    TrtDetector() = default;
    ~TrtDetector() { shutdown(); }

    TrtDetector(const TrtDetector&) = delete;
    TrtDetector& operator=(const TrtDetector&) = delete;

    bool init(const std::string& engine_path);
    bool infer(const cv::Mat& bgr_frame,
               std::vector<BoundingBox>& detections_out);
    void shutdown();
    bool is_initialized() const;

private:
    class Logger : public nvinfer1::ILogger {
    public:
        void log(Severity /*severity*/, const char* /*msg*/) noexcept override {
        }
    };

    bool preprocess(const cv::Mat& bgr_frame);
    bool postprocess(std::vector<BoundingBox>& detections_out,
                     int32_t orig_w, int32_t orig_h, int64_t timestamp_ns);
    static float32_t iou(const BoundingBox& a, const BoundingBox& b);
    static void      nms(std::vector<BoundingBox>& dets, float32_t thresh);

    Logger        logger_;
    RuntimePtr    runtime_ {nullptr, [](nvinfer1::IRuntime* p)   { delete p; }};
    EnginePtr     engine_  {nullptr, [](nvinfer1::ICudaEngine* p) { delete p; }};
    ContextPtr    context_ {nullptr, [](nvinfer1::IExecutionContext* p) { delete p; }};
    CudaStreamPtr stream_;

    HostBufPtr   input_host_;
    DeviceBufPtr input_dev_;
    HostBufPtr   output_host_;
    DeviceBufPtr output_dev_;

    std::size_t input_bytes_  = 0U;
    std::size_t output_bytes_ = 0U;
    bool        initialized_  = false;
    bool        range_warned_ = false;

    const char* input_name_  = nullptr;
    const char* output_name_ = nullptr;
};

} // namespace cuas
