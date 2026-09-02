// @file auto_exposure.hpp
// @brief Two-stage auto-exposure controller math (exposure first, then gain).
//
// Pure logic — the node owns the V4L2 ioctls. Brightness is served with
// exposure time up to a motion-blur ceiling (exposure_max, deliberately
// below the sensor's 65535 limit: a fast crossing target must not smear,
// OVERHAUL P4.4), then with analog gain; when the scene brightens, gain
// returns to minimum before exposure drops, so noise is only paid when
// exposure alone cannot reach the target.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

struct AeLimits {
    int32_t exposure_min = 2;
    int32_t exposure_max = 8000;
    int32_t gain_min     = 100;
    int32_t gain_max     = 1200;
};

struct AeParams {
    float32_t target_mean   = 110.0F;  // 8-bit luma setpoint
    float32_t deadband      = 10.0F;   // no action within +/- of target
    float32_t max_step_frac = 0.25F;   // per-update multiplicative cap
    AeLimits  limits{};
};

struct AeCommand {
    int32_t exposure = 0;
    int32_t gain     = 0;
    bool    changed  = false;  // true iff either output differs from input
};

// One controller update. Inputs outside the limits are pulled into range
// (enforces the motion-blur ceiling on an externally-set exposure); a
// non-finite mean_luma holds the current settings.
AeCommand nextAeCommand(int32_t exposure, int32_t gain,
                        float32_t mean_luma, const AeParams& params);

} // namespace cuas
