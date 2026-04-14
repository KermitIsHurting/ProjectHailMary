// @file clock.hpp
// @brief Monotonic nanosecond timestamp helper.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <time.h>

namespace cuas {

inline int64_t now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL
         + static_cast<int64_t>(ts.tv_nsec);
}

} // namespace cuas
