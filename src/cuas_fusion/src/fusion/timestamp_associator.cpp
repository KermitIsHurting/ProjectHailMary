// @file timestamp_associator.cpp
// @brief Ring-buffer camera frame nearest-neighbour lookup by timestamp.
#include "cuas_fusion/fusion/timestamp_associator.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <cstdlib>

namespace cuas {

void TimestampAssociator::addCameraFrame(const cv::Mat& frame, int64_t timestamp_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // copyTo reuses the ring slot's buffer once allocated (clone() allocated
    // ~6 MB per frame at 30 Hz, A3.5). Steady state: zero allocations.
    frame.copyTo(buffer_[head_].frame);
    buffer_[head_].timestamp_ns = timestamp_ns;
    head_ = (head_ + 1U) % TIMESTAMP_BUFFER_SIZE;
    if (count_ < TIMESTAMP_BUFFER_SIZE) {
        ++count_;
    }
}

bool TimestampAssociator::findBestMatch(int64_t radar_ts_ns,
                                        cv::Mat& out_frame,
                                        int64_t& out_ts_ns) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0U) {
        return false;
    }

    std::size_t best = 0U;
    int64_t min_delta = std::abs(buffer_[0].timestamp_ns - radar_ts_ns);

    for (std::size_t i = 1U; i < count_; ++i) {
        const int64_t delta = std::abs(buffer_[i].timestamp_ns - radar_ts_ns);
        if (delta < min_delta) {
            min_delta = delta;
            best = i;
        }
    }

    if (min_delta > MAX_TIMESTAMP_DELTA_NS) {
        return false;
    }

    // Deep copy into the caller's (reusable) Mat: a shallow share would be
    // overwritten in place when the ring slot recycles under copyTo.
    buffer_[best].frame.copyTo(out_frame);
    out_ts_ns = buffer_[best].timestamp_ns;
    return true;
}

} // namespace cuas
