// @file camera_driver.hpp
// @brief V4L2 MIPI CSI camera driver wrapping VIDIOC ioctls and mmap buffers.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace cuas {

static constexpr int32_t V4L2_BUF_COUNT = 4;

class CameraDriver {
public:
    CameraDriver();
    ~CameraDriver();

    CameraDriver(const CameraDriver &) = delete;
    CameraDriver & operator=(const CameraDriver &) = delete;

    bool open(const std::string & device_path);
    bool grabFrame(cv::Mat & out_bgr, int64_t & timestamp_ns);
    void close();

private:
    int32_t  fd_;
    void *   buffers_[V4L2_BUF_COUNT];
    uint32_t buf_lengths_[V4L2_BUF_COUNT];
    bool     streaming_;

    // Per-frame scratch Mats, allocated once in open(): subtract/cvtColor/
    // split/convertTo reuse a matching destination, so the ~5 full-frame
    // allocations per grabFrame drop to zero (R12e). Pixel math unchanged.
    cv::Mat raw_sub_;
    cv::Mat bgr16_;
    cv::Mat channels16_[3];
    cv::Mat channels8_[3];
};

} // namespace cuas
