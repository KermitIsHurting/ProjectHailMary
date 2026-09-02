// @file eigen_types.hpp
// @brief Fixed-size Eigen aliases for the estimation stack.
//
// Every estimation dimension is known at compile time: 6 (shared pos+vel
// space), 9 (CA state), 7 (CT state), 3 (position measurement). Fixed sizes
// keep the per-scan hot path free of heap allocation (A3.1, DEV-005).
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstddef>

namespace cuas {

using Vector6d = Eigen::Matrix<float64_t, 6, 1>;
using Matrix6d = Eigen::Matrix<float64_t, 6, 6>;
using Vector7d = Eigen::Matrix<float64_t, 7, 1>;
using Matrix7d = Eigen::Matrix<float64_t, 7, 7>;
using Vector9d = Eigen::Matrix<float64_t, 9, 1>;
using Matrix9d = Eigen::Matrix<float64_t, 9, 9>;

// Track.msg covariance wire format (P3.1): row-major upper triangle of
// the 6x6 (position, velocity) covariance — 21 values, one packer so the
// legacy and central trackers can never disagree on the ordering.
inline void packUpperTriangle6(const Matrix6d& P,
                               std::array<float64_t, 21>& out)
{
    std::size_t k = 0U;
    for (Eigen::Index i = 0; i < 6; ++i) {
        for (Eigen::Index j = i; j < 6; ++j) {
            out[k] = P(i, j);
            ++k;
        }
    }
}

} // namespace cuas
