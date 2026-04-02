// camera_driver.hpp
// Low-level V4L2 camera capture with mmap: opens Tegra CSI device, grabs
// raw BA10 Bayer frames, demosaics to BGR, timestamps with CLOCK_MONOTONIC.
// Zero ROS dependency — usable in standalone tests.

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <cstdint>

namespace cuas {

static constexpr int V4L2_BUF_COUNT = 4;

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
    int fd_;
    void * buffers_[V4L2_BUF_COUNT];
    uint32_t buf_lengths_[V4L2_BUF_COUNT];
    bool streaming_;
};

} // namespace cuas
