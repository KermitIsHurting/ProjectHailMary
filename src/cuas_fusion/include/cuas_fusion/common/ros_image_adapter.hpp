// @file ros_image_adapter.hpp
// @brief Non-throwing sensor_msgs::msg::Image to cv::Mat adapter.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace cuas {

// Returned cv::Mat references msg's pixel buffer; valid only while msg is alive.
inline bool rosImageToBgr(const sensor_msgs::msg::Image& msg, cv::Mat& out_bgr)
{
    if (msg.width == 0U || msg.height == 0U) {
        return false;
    }

    const int32_t rows = static_cast<int32_t>(msg.height);
    const int32_t cols = static_cast<int32_t>(msg.width);

    if (msg.encoding == "bgr8") {
        out_bgr = cv::Mat(rows, cols, CV_8UC3,
                          const_cast<uint8_t*>(msg.data.data()),
                          static_cast<std::size_t>(msg.step));
        return !out_bgr.empty();
    }

    if (msg.encoding == "rgb8") {
        const cv::Mat rgb(rows, cols, CV_8UC3,
                          const_cast<uint8_t*>(msg.data.data()),
                          static_cast<std::size_t>(msg.step));
        cv::cvtColor(rgb, out_bgr, cv::COLOR_RGB2BGR);
        return !out_bgr.empty();
    }

    return false;
}

}  // namespace cuas
