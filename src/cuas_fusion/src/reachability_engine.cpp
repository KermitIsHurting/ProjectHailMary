// @file reachability_engine.cpp
// @brief Implements intercept-time and 2x2 analytic eigen-decomposition.
#include "cuas_fusion/reachability_engine.hpp"

#include <cmath>

namespace cuas {

InterceptResult ReachabilityEngine::compute(
    const ReachabilityTrackState& state) const
{
    InterceptResult result;
    result.time_to_intercept_s    = 0.0F;
    result.intercept_confidence   = 0.0F;
    result.ellipse_major_m        = 0.0F;
    result.ellipse_minor_m        = 0.0F;
    result.ellipse_heading_rad    = 0.0F;
    result.intercept_possible     = false;

    float32_t conf = state.imm_cv_weight;
    if (conf < 0.0F) {
        conf = 0.0F;
    }
    if (conf > 1.0F) {
        conf = 1.0F;
    }
    result.intercept_confidence = conf;

    const float32_t dist_sq = (state.x_m * state.x_m) + (state.y_m * state.y_m);
    const float32_t distance = std::sqrt(dist_sq);

    if (distance > 0.0F) {
        const float32_t bx = -state.x_m / distance;
        const float32_t by = -state.y_m / distance;
        const float32_t speed_toward =
            (state.vx_mps * bx) + (state.vy_mps * by);

        if (speed_toward > 0.0F) {
            result.time_to_intercept_s = distance / speed_toward;
            result.intercept_possible  = true;
        }
    }

    const float32_t a = state.P[0][0];
    const float32_t b = state.P[0][1];
    const float32_t d = state.P[1][1];
    const float32_t trace_half = (a + d) * 0.5F;
    const float32_t det        = (a * d) - (b * b);
    const float32_t radicand   = (trace_half * trace_half) - det;
    float32_t sqrt_disc = 0.0F;
    if (radicand > 0.0F) {
        sqrt_disc = std::sqrt(radicand);
    }

    const float32_t lambda_max = trace_half + sqrt_disc;
    const float32_t lambda_min = trace_half - sqrt_disc;
    float32_t lmax_clamped = 0.0F;
    if (lambda_max > 0.0F) {
        lmax_clamped = lambda_max;
    }
    float32_t lmin_clamped = 0.0F;
    if (lambda_min > 0.0F) {
        lmin_clamped = lambda_min;
    }
    result.ellipse_major_m = std::sqrt(lmax_clamped);
    result.ellipse_minor_m = std::sqrt(lmin_clamped);

    float32_t ev_x = 0.0F;
    float32_t ev_y = 0.0F;
    if (b != 0.0F) {
        ev_x = lambda_max - d;
        ev_y = b;
    } else {
        if (a >= d) {
            ev_x = 1.0F;
            ev_y = 0.0F;
        } else {
            ev_x = 0.0F;
            ev_y = 1.0F;
        }
    }
    result.ellipse_heading_rad = std::atan2(ev_y, ev_x);

    return result;
}

} // namespace cuas
