#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace cuas {

class TrackManager {
public:
    TrackManager() = default;

    bool init();

    // Associate detections with existing tracks; create/promote/expire as needed.
    // Fills confirmed_out with all CONFIRMED tracks after update.
    bool update(const std::vector<FusedDetection>& detections,
                std::vector<Track>& confirmed_out);

private:
    struct TrackEntry {
        Track track;
        int   hit_count  = 0;
        int   miss_count = 0;
        bool  active     = false;
    };

    // Find index of nearest active track within TRACK_ASSOCIATION_DIST_M.
    // Returns TRACK_MAX_TRACKS if none found.
    size_t findNearest(float x, float y, float z) const;

    // Allocate the next free slot; returns TRACK_MAX_TRACKS if full.
    size_t allocateSlot();

    std::array<TrackEntry, TRACK_MAX_TRACKS> entries_{};
    uint32_t next_id_ = 1;
};

} // namespace cuas
