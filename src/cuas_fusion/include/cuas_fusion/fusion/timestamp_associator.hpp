// timestamp_associator.hpp
// Aligns camera frames with radar detections by finding the camera frame
// whose CLOCK_MONOTONIC timestamp is nearest to a given radar timestamp.

#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <opencv2/core.hpp>

#include "cuas_fusion/common/constants.hpp"

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
    size_t head_  = 0;
    size_t count_ = 0;
    mutable std::mutex mutex_;
};

} // namespace cuas
