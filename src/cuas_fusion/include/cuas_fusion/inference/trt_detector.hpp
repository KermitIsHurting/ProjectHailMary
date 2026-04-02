// trt_detector.hpp
// TensorRT-accelerated YOLOv8 detector: loads a serialized engine, runs
// inference on BGR camera frames, returns bounding-box detections.
// Zero ROS dependency — usable in standalone tests and benchmarks.

#pragma once

#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>

namespace cuas {

// ---------------------------------------------------------------------------
// CUDA / TensorRT custom deleters (destroy via delete, TRT v10 uses dtors)
// ---------------------------------------------------------------------------
struct CudaStreamDeleter {
    void operator()(cudaStream_t s) const {
        if (s) cudaStreamDestroy(s);
    }
};

struct CudaFreeDeleter {
    void operator()(void* p) const {
        if (p) cudaFree(p);
    }
};

struct CudaFreeHostDeleter {
    void operator()(void* p) const {
        if (p) cudaFreeHost(p);
    }
};

using RuntimePtr  = std::unique_ptr<nvinfer1::IRuntime,  void(*)(nvinfer1::IRuntime*)>;
using EnginePtr   = std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)>;
using ContextPtr  = std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)>;
using CudaStreamPtr = std::unique_ptr<std::remove_pointer<cudaStream_t>::type, CudaStreamDeleter>;
using DeviceBufPtr  = std::unique_ptr<void, CudaFreeDeleter>;
using HostBufPtr    = std::unique_ptr<void, CudaFreeHostDeleter>;

// ---------------------------------------------------------------------------
// TrtDetector
// ---------------------------------------------------------------------------
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
    // TensorRT logger (minimal, stderr-only)
    class Logger : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override {
            if (severity <= Severity::kWARNING) {
                fprintf(stderr, "[TRT %d] %s\n", static_cast<int>(severity), msg);
            }
        }
    };

    bool preprocess(const cv::Mat& bgr_frame);
    bool postprocess(std::vector<BoundingBox>& detections_out,
                     int orig_w, int orig_h, int64_t timestamp_ns);
    static float iou(const BoundingBox& a, const BoundingBox& b);
    static void  nms(std::vector<BoundingBox>& dets, float thresh);

    Logger       logger_;
    RuntimePtr   runtime_  {nullptr, [](nvinfer1::IRuntime* p)  { delete p; }};
    EnginePtr    engine_   {nullptr, [](nvinfer1::ICudaEngine* p) { delete p; }};
    ContextPtr   context_  {nullptr, [](nvinfer1::IExecutionContext* p) { delete p; }};
    CudaStreamPtr stream_;

    // Pre-allocated buffers
    HostBufPtr   input_host_;                       // pinned: 1x3x640x640 float
    DeviceBufPtr input_dev_;                        // device: same
    HostBufPtr   output_host_;                      // pinned: 1x84x8400 float
    DeviceBufPtr output_dev_;                       // device: same

    size_t input_bytes_  = 0;
    size_t output_bytes_ = 0;
    bool   initialized_  = false;
    bool   range_warned_ = false;                   // one-shot normalization warning

    // Tensor names discovered from engine
    const char* input_name_  = nullptr;
    const char* output_name_ = nullptr;
};

} // namespace cuas
