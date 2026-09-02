// @file trt_detector.hpp
// @brief TensorRT YOLO object detector wrapper.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <array>
#include <cstdio>
#include <memory>
#include <string>

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

// Stateless functor deleter: a function-pointer deleter doubles the smart
// pointer's size and can be default-constructed null (dtor UB path).
struct TrtDeleter {
    void operator()(nvinfer1::IRuntime* p) const { delete p; }
    void operator()(nvinfer1::ICudaEngine* p) const { delete p; }
    void operator()(nvinfer1::IExecutionContext* p) const { delete p; }
};

using RuntimePtr    = std::unique_ptr<nvinfer1::IRuntime, TrtDeleter>;
using EnginePtr     = std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter>;
using ContextPtr    = std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter>;
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
               FixedVector<BoundingBox, 128U>& detections_out);
    void shutdown();
    bool is_initialized() const;

private:
    class Logger : public nvinfer1::ILogger {
    public:
        // TensorRT reports engine-load failures only through this callback;
        // discarding them made every failure an anonymous nullptr return.
        void log(Severity severity, const char* msg) noexcept override {
            if (severity <= Severity::kERROR) {
                error_seen_ = true;
                std::fprintf(stderr, "[TensorRT] %s\n", msg);
            }
        }
        bool errorSeen() const { return error_seen_; }

    private:
        bool error_seen_ = false;
    };

    bool preprocess(const cv::Mat& bgr_frame);
    bool postprocess(FixedVector<BoundingBox, 128U>& detections_out,
                     int32_t orig_w, int32_t orig_h, int64_t timestamp_ns);
    static float32_t iou(const BoundingBox& a, const BoundingBox& b);
    static void      nms(FixedVector<BoundingBox, 128U>& dets, float32_t thresh);

    Logger        logger_;
    RuntimePtr    runtime_;
    EnginePtr     engine_;
    ContextPtr    context_;
    CudaStreamPtr stream_;

    HostBufPtr   input_host_;
    DeviceBufPtr input_dev_;
    HostBufPtr   output_host_;
    DeviceBufPtr output_dev_;

    // Preprocess scratch buffers, created once at init(): resize/convertTo/
    // split reuse a matching destination, so the four full-frame Mats that
    // were allocated per frame drop to zero steady-state allocations (A3.4).
    cv::Mat preproc_resized_;
    cv::Mat preproc_float_;
    std::array<cv::Mat, 3> preproc_channels_;

    std::size_t input_bytes_  = 0U;
    std::size_t output_bytes_ = 0U;
    bool        initialized_  = false;
    bool        range_warned_ = false;

    const char* input_name_  = nullptr;
    const char* output_name_ = nullptr;
};

} // namespace cuas
