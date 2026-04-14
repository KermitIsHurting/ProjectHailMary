// @file timestamp_associator.hpp
// @brief Ring-buffer camera-frame lookup matched to radar timestamps.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <array>
#include <mutex>
#include <opencv2/core.hpp>

namespace cuas {

class TimestampAssociator {
public:
    TimestampAssociator() = default;

    void addCameraFrame(const cv::Mat& frame, int64_t timestamp_ns);
    bool findBestMatch(int64_t radar_ts_ns, cv::Mat& out_frame, int64_t& out_ts_ns) const;

private:
    struct Entry {
        cv::Mat frame;
        int64_t timestamp_ns = 0;
    };

    std::array<Entry, TIMESTAMP_BUFFER_SIZE> buffer_{};
    std::size_t head_  = 0U;
    std::size_t count_ = 0U;
    mutable std::mutex mutex_;
};

} // namespace cuas
