// @file reachability_engine.hpp
// @brief Closed-form intercept time and 2x2 covariance ellipse extraction.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

// WHY: named ReachabilityTrackState to avoid collision with the pipeline-wide
// enum class TrackState declared in cuas_fusion/common/types.hpp.
struct ReachabilityTrackState {
    float32_t x_m;
    float32_t y_m;
    float32_t vx_mps;
    float32_t vy_mps;
    float32_t P[4][4];
    float32_t imm_cv_weight;
};

struct InterceptResult {
    float32_t time_to_intercept_s;
    float32_t intercept_confidence;
    float32_t ellipse_major_m;
    float32_t ellipse_minor_m;
    float32_t ellipse_heading_rad;
    bool      intercept_possible;
};

class ReachabilityEngine {
public:
    ReachabilityEngine() = default;

    InterceptResult compute(const ReachabilityTrackState& state) const;
};

} // namespace cuas
