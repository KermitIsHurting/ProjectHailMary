// @file detection_set_buffer.hpp
// @brief Ring buffer of stamped YOLO detection sets for stamp-based fusion.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <array>
#include <cstddef>

namespace cuas {

// The TimestampAssociator pattern (fixed ring + nearest-stamp lookup)
// applied to detection SETS instead of image frames: fusion selects the
// camera observation nearest a track stamp instead of "latest box within
// 500 ms of arrival" (P2.2). An EMPTY set is a valid entry — it records
// that the frame really contained no targets, so selecting it correctly
// yields no label fusion at that instant (A1.7 semantics preserved).
//
// Not internally synchronized: the owning node guards access with its
// callback mutex (single-threaded executor today, DEV-010).
class DetectionSetBuffer {
public:
    using BoxSet = FixedVector<BoundingBox, FUSION_MAX_DETECTIONS>;

    void addSet(const BoxSet& boxes, int64_t stamp_ns)
    {
        entries_[head_].boxes    = boxes;
        entries_[head_].stamp_ns = stamp_ns;
        entries_[head_].valid    = true;
        head_ = (head_ + 1U) % TIMESTAMP_BUFFER_SIZE;
    }

    // Copies the buffered set nearest target_ns into out, provided its
    // stamp distance is within max_delta_ns (inclusive — the window edge
    // is a match, mirroring TimestampAssociator). Returns false and
    // leaves the outputs untouched when nothing qualifies.
    bool selectNearest(int64_t target_ns, int64_t max_delta_ns,
                       BoxSet& out, int64_t& out_stamp_ns) const
    {
        const Entry* best       = nullptr;
        int64_t      best_delta = max_delta_ns;
        for (const Entry& e : entries_) {
            if (!e.valid) {
                continue;
            }
            const int64_t d = (e.stamp_ns > target_ns)
                                  ? (e.stamp_ns - target_ns)
                                  : (target_ns - e.stamp_ns);
            if (d <= best_delta) {
                best_delta = d;
                best       = &e;
            }
        }
        if (best == nullptr) {
            return false;
        }
        out          = best->boxes;
        out_stamp_ns = best->stamp_ns;
        return true;
    }

private:
    struct Entry {
        BoxSet  boxes{};
        int64_t stamp_ns = 0;
        bool    valid    = false;
    };

    std::array<Entry, TIMESTAMP_BUFFER_SIZE> entries_{};
    std::size_t head_ = 0U;
};

} // namespace cuas
