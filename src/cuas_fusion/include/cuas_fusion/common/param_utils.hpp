// @file param_utils.hpp
// @brief Range validation for user-supplied node parameters.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace cuas {

// Clamp a user-supplied publish rate to a sane range before deriving a
// timer period. rate <= 0 makes 1000/rate non-finite, and casting a
// non-finite double to int is undefined behavior ([conv.fpint]); a rate
// above max_hz truncates the period to 0 ms (busy-spin timer). Negated
// comparisons so NaN also takes the fallback branch.
inline float64_t clamp_rate_hz(const rclcpp::Logger& logger,
                               const char* param_name,
                               float64_t rate,
                               float64_t fallback_hz,
                               float64_t min_hz = 0.1,
                               float64_t max_hz = 100.0)
{
    if (!(rate >= min_hz) || !(rate <= max_hz)) {
        RCLCPP_WARN(logger,
                    "%s=%.3f outside [%.1f, %.1f] Hz; using fallback %.1f Hz",
                    param_name, rate, min_hz, max_hz, fallback_hz);
        return fallback_hz;
    }
    return rate;
}

// Validate a horizon/step pair and derive the forward-propagation step
// count. step_dt <= 0 makes horizon/step_dt non-finite and the int cast
// undefined behavior ([conv.fpint]); steps beyond max_steps would be
// computed and then discarded by the fixed trajectory buffers. Negated
// comparisons so NaN also takes the fallback branch. horizon_sec and
// step_dt_sec are corrected in place when out of range.
inline int32_t clamp_prediction_steps(const rclcpp::Logger& logger,
                                      float64_t& horizon_sec,
                                      float64_t& step_dt_sec,
                                      uint32_t max_steps)
{
    if (!(horizon_sec > 0.0) || !(horizon_sec <= 60.0)) {
        RCLCPP_WARN(logger,
                    "prediction_horizon_sec=%.3f outside (0, 60]; using 5.0",
                    horizon_sec);
        horizon_sec = 5.0;
    }
    if (!(step_dt_sec > 0.0) || !(step_dt_sec <= horizon_sec)) {
        RCLCPP_WARN(logger,
                    "prediction_step_dt=%.3f outside (0, horizon]; using 0.1",
                    step_dt_sec);
        step_dt_sec = 0.1;
    }

    int32_t n_steps = static_cast<int32_t>(horizon_sec / step_dt_sec);
    if (n_steps < 1) {
        n_steps = 1;
    }
    if (n_steps > static_cast<int32_t>(max_steps)) {
        RCLCPP_WARN(logger,
                    "horizon/step_dt = %d steps exceeds the %u-step trajectory "
                    "buffer; clamping (increase prediction_step_dt to cover "
                    "the full horizon)",
                    n_steps, max_steps);
        n_steps = static_cast<int32_t>(max_steps);
    }
    return n_steps;
}

}  // namespace cuas
