// timestamp_associator.cpp
// Implements nearest-neighbor camera-frame lookup for radar timestamp association.
// Zero dynamic allocation — uses a fixed circular buffer of size TIMESTAMP_BUFFER_SIZE.

#include "cuas_fusion/fusion/timestamp_associator.hpp"

#include <cstdio>
#include <cstdlib>

namespace cuas {

void TimestampAssociator::addCameraFrame(const cv::Mat& frame, int64_t timestamp_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[head_].frame = frame.clone();
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
    if (count_ == 0) {
        return false;
    }

    size_t best = 0;
    int64_t min_delta = std::abs(buffer_[0].timestamp_ns - radar_ts_ns);

    for (size_t i = 1; i < count_; ++i) {
        const int64_t delta = std::abs(buffer_[i].timestamp_ns - radar_ts_ns);
        if (delta < min_delta) {
            min_delta = delta;
            best = i;
        }
    }

    if (min_delta > MAX_TIMESTAMP_DELTA_NS) {
        fprintf(stderr,
                "[TimestampAssociator] WARNING: best match delta %ldns exceeds 50ms limit\n",
                static_cast<long>(min_delta));
        return false;
    }

    out_frame = buffer_[best].frame;
    out_ts_ns = buffer_[best].timestamp_ns;
    return true;
}

} // namespace cuas
