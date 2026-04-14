// @file camera_driver.cpp
// @brief V4L2 MIPI CSI capture to BGR conversion pipeline.
#include "cuas_fusion/drivers/camera_driver.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <ctime>

// BA10 fourcc (10-bit Bayer GRGR/BGBG)
#ifndef V4L2_PIX_FMT_SGRBG10
#define V4L2_PIX_FMT_SGRBG10 v4l2_fourcc('B', 'A', '1', '0')
#endif

namespace cuas {

CameraDriver::CameraDriver()
    : fd_(-1), buffers_{}, buf_lengths_{}, streaming_(false)
{
}

CameraDriver::~CameraDriver()
{
    close();
}

bool CameraDriver::open(const std::string & device_path)
{
    fd_ = ::open(device_path.c_str(), O_RDWR);
    if (fd_ < 0) {
        return false;
    }

    struct v4l2_format fmt{};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAMERA_WIDTH;
    fmt.fmt.pix.height      = CAMERA_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SGRBG10;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    struct v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = CAMERA_FPS;
    ioctl(fd_, VIDIOC_S_PARM, &parm);  // best-effort

    struct v4l2_requestbuffers req{};
    req.count  = V4L2_BUF_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    for (uint32_t i = 0; i < req.count && i < V4L2_BUF_COUNT; ++i) {
        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            close();
            return false;
        }

        buf_lengths_[i] = buf.length;
        buffers_[i] = mmap(nullptr, buf.length,
                           PROT_READ | PROT_WRITE, MAP_SHARED,
                           fd_, buf.m.offset);

        if (buffers_[i] == MAP_FAILED) {
            buffers_[i] = nullptr;
            close();
            return false;
        }

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            close();
            return false;
        }
    }

    int32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        close();
        return false;
    }
    streaming_ = true;

    return true;
}

bool CameraDriver::grabFrame(cv::Mat & out_bgr, int64_t & timestamp_ns)
{
    if (fd_ < 0 || !streaming_) {
        return false;
    }

    struct v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        return false;
    }

    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    timestamp_ns = static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL
                 + static_cast<int64_t>(ts.tv_nsec);

    // BA10 is 10-bit data packed in 16-bit words — wrap as CV_16UC1 without copy
    cv::Mat raw16(CAMERA_HEIGHT, CAMERA_WIDTH, CV_16UC1,
                  buffers_[buf.index]);

    cv::Mat raw_sub;
    cv::subtract(raw16, cv::Scalar(CAMERA_BLACK_LEVEL), raw_sub);

    cv::Mat bgr16;
    cv::cvtColor(raw_sub, bgr16, cv::COLOR_BayerGB2BGR);

    // Per-channel WB gains convert black-subtracted 16-bit Bayer down to 8-bit BGR
    cv::Mat channels[3];
    cv::split(bgr16, channels);
    channels[0].convertTo(channels[0], CV_8UC1, CAMERA_WB_GAIN_B * CAMERA_TONE_SCALE);
    channels[1].convertTo(channels[1], CV_8UC1, CAMERA_WB_GAIN_G * CAMERA_TONE_SCALE);
    channels[2].convertTo(channels[2], CV_8UC1, CAMERA_WB_GAIN_R * CAMERA_TONE_SCALE);
    cv::merge(channels, 3, out_bgr);

    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        return false;
    }

    return true;
}

void CameraDriver::close()
{
    if (fd_ < 0) {
        return;
    }

    if (streaming_) {
        int32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }

    for (int32_t i = 0; i < V4L2_BUF_COUNT; ++i) {
        if (buffers_[i] && buffers_[i] != MAP_FAILED) {
            munmap(buffers_[i], buf_lengths_[i]);
            buffers_[i] = nullptr;
        }
    }

    ::close(fd_);
    fd_ = -1;
}

} // namespace cuas
