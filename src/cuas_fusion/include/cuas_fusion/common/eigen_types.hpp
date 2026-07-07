// @file eigen_types.hpp
// @brief Fixed-size Eigen aliases for the estimation stack.
//
// Every estimation dimension is known at compile time: 6 (shared pos+vel
// space), 9 (CA state), 7 (CT state), 3 (position measurement). Fixed sizes
// keep the per-scan hot path free of heap allocation (A3.1, DEV-005).
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

namespace cuas {

using Vector6d = Eigen::Matrix<float64_t, 6, 1>;
using Matrix6d = Eigen::Matrix<float64_t, 6, 6>;
using Vector7d = Eigen::Matrix<float64_t, 7, 1>;
using Matrix7d = Eigen::Matrix<float64_t, 7, 7>;
using Vector9d = Eigen::Matrix<float64_t, 9, 1>;
using Matrix9d = Eigen::Matrix<float64_t, 9, 9>;

} // namespace cuas
