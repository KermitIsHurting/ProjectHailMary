// @file auto_exposure.cpp
// @brief Two-stage auto-exposure controller (exposure first, then gain).
#include "cuas_fusion/drivers/auto_exposure.hpp"

#include <cmath>

namespace cuas {

namespace {

int32_t clampi(const int32_t v, const int32_t lo, const int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Multiplicative step toward the target; guarantees at least +/-1 count of
// motion when rounding would stall the loop outside the deadband.
int32_t stepToward(const int32_t value, const float32_t factor,
                   const bool brighter, const int32_t lo, const int32_t hi)
{
    const auto scaled = static_cast<int32_t>(
        std::lround(static_cast<float64_t>(value) *
                    static_cast<float64_t>(factor)));
    int32_t next = clampi(scaled, lo, hi);
    if (next == value) {
        next = clampi(value + (brighter ? 1 : -1), lo, hi);
    }
    return next;
}

} // namespace

AeCommand nextAeCommand(const int32_t exposure, const int32_t gain,
                        const float32_t mean_luma, const AeParams& params)
{
    AeCommand out;
    out.exposure = clampi(exposure,
                          params.limits.exposure_min,
                          params.limits.exposure_max);
    out.gain = clampi(gain, params.limits.gain_min, params.limits.gain_max);

    // Negated comparisons: a NaN mean takes the hold branch.
    const bool mean_valid = (mean_luma >= 0.0F) && (mean_luma <= 255.0F);
    const float32_t err = params.target_mean - mean_luma;
    if (mean_valid && (std::fabs(err) > params.deadband)) {
        float32_t frac = err / params.target_mean;
        if (frac > params.max_step_frac) {
            frac = params.max_step_frac;
        }
        if (frac < -params.max_step_frac) {
            frac = -params.max_step_frac;
        }
        const float32_t factor = 1.0F + frac;
        const bool brighter = (err > 0.0F);
        if (brighter) {
            // Too dark: exposure carries the load until its ceiling.
            if (out.exposure < params.limits.exposure_max) {
                out.exposure = stepToward(out.exposure, factor, true,
                                          params.limits.exposure_min,
                                          params.limits.exposure_max);
            } else {
                out.gain = stepToward(out.gain, factor, true,
                                      params.limits.gain_min,
                                      params.limits.gain_max);
            }
        } else {
            // Too bright: shed gain (noise) before shortening exposure.
            if (out.gain > params.limits.gain_min) {
                out.gain = stepToward(out.gain, factor, false,
                                      params.limits.gain_min,
                                      params.limits.gain_max);
            } else {
                out.exposure = stepToward(out.exposure, factor, false,
                                          params.limits.exposure_min,
                                          params.limits.exposure_max);
            }
        }
    }
    out.changed = (out.exposure != exposure) || (out.gain != gain);
    return out;
}

} // namespace cuas
