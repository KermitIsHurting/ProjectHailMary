// @file track.hpp
// @brief Track data structure carried through the tracking pipeline.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <string>

namespace cuas {

class Track {
public:
    Track() = default;

    uint32_t    track_id_      = 0U;
    float32_t   position_x_m_  = 0.0F;
    float32_t   position_y_m_  = 0.0F;
    float32_t   position_z_m_  = 0.0F;
    float32_t   velocity_mps_  = 0.0F;
    float32_t   doppler_mps_   = 0.0F;  // negative = approaching
    int32_t     class_id_      = -1;    // -1 = unlabeled (A3.8)
    float32_t   confidence_   = 0.0F;
    TrackState  state_        = TrackState::TENTATIVE;
    int64_t     timestamp_ns_ = 0;
};

} // namespace cuas
