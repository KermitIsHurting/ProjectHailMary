// @file label_join.hpp
// @brief Camera-label join: nearest fused detection to a track, time-corrected (D-10, R6 F1).
#pragma once

#include "cuas_fusion/common/bearing.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <cuas_msgs/msg/fused_detection.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace cuas {

struct LabelJoinTrack {
    float32_t x_m  = 0.0F;
    float32_t y_m  = 0.0F;
    float32_t z_m  = 0.0F;
    float32_t vx   = 0.0F;
    float32_t vy   = 0.0F;
    float32_t vz   = 0.0F;
    int64_t   stamp_ns = 0;   // the track's measurement time
};

// Fused positions are valid at the camera instant (fused_ns); the track is
// valid at its own stamp. Compare like with like: extrapolate the track to
// fused_ns first, else a 10 m/s target 100 ms apart sits 1 m from its own
// label and never gets it (R6 F1). Returns the index of the matched
// detection or -1. Extrapolation is clamped to ±kMaxExtrapS.
// Seq is any sequence of FusedDetection with size() and operator[]: the
// wire type is a rosidl BoundedVector, tests use std::vector.
template <typename Seq>
inline int32_t joinFusedLabel(const LabelJoinTrack& t,
                              const Seq& dets,
                              int64_t fused_ns,
                              float32_t max_dist_m,
                              float32_t max_bearing_deg)
{
    constexpr float32_t kMaxExtrapS = 0.5F;
    const float32_t dt_s = std::clamp(
        static_cast<float32_t>(fused_ns - t.stamp_ns) * 1.0e-9F, -kMaxExtrapS, kMaxExtrapS);
    const float32_t px = t.x_m + (t.vx * dt_s);
    const float32_t py = t.y_m + (t.vy * dt_s);
    const float32_t pz = t.z_m + (t.vz * dt_s);
    const float32_t track_az = bearingDegBoresightZero(px, py);

    int32_t   best = -1;
    float32_t best_dist = max_dist_m;
    for (std::size_t di = 0U; di < dets.size(); ++di) {
        const cuas_msgs::msg::FusedDetection& fd = dets[di];
        const float32_t dx = fd.position_x_m - px;
        const float32_t dy = fd.position_y_m - py;
        const float32_t dz = fd.position_z_m - pz;
        const float32_t dist = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
        // Wrap the bearing difference at +/-180 deg: +179 vs -179 is 2 deg
        // apart, not 358.
        float32_t diff = std::fmod(std::abs(fd.azimuth_deg - track_az), 360.0F);
        if (diff > 180.0F) {
            diff = 360.0F - diff;
        }
        if ((dist <= best_dist) && (diff < max_bearing_deg)) {
            best_dist = dist;
            best      = static_cast<int32_t>(di);
        }
    }
    return best;
}

} // namespace cuas
