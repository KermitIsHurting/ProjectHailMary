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

}  // namespace cuas
