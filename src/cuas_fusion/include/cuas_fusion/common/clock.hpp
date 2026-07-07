// @file clock.hpp
// @brief Monotonic nanosecond timestamp helper.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <ctime>

namespace cuas {

// Every timestamp in the pipeline flows through here. The return of
// clock_gettime must be checked: on the (theoretical) failure path ts would
// otherwise be read uninitialized — MISRA C++:2023 11.6.2 is Mandatory and
// admits no deviation, so the guard is obligatory even though
// CLOCK_MONOTONIC cannot realistically fail on Linux.
inline int64_t now_ns()
{
    struct timespec ts = {};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;  // explicit recovery: callers treat 0 as "no valid time"
    }
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL
         + static_cast<int64_t>(ts.tv_nsec);
}

} // namespace cuas
