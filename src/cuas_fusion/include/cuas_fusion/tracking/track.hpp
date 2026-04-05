#pragma once

#include "cuas_fusion/common/types.hpp"

#include <cstdint>
#include <string>

namespace cuas {

class Track {
public:
    Track() = default;

    uint32_t    track_id_       = 0;
    float       position_x_m_  = 0.0f;
    float       position_y_m_  = 0.0f;
    float       position_z_m_  = 0.0f;
    float       velocity_mps_  = 0.0f;  // speed magnitude
    float       doppler_mps_   = 0.0f;  // radial velocity; negative = approaching
    std::string class_label_;
    float       confidence_    = 0.0f;
    TrackState  state_         = TrackState::TENTATIVE;
    int64_t     timestamp_ns_  = 0;
};

} // namespace cuas
