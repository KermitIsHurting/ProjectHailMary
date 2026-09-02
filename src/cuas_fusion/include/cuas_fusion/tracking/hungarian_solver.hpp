// @file hungarian_solver.hpp
// @brief Jonker-Volgenant assignment solver with fixed, pre-allocated buffers.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

class HungarianSolver {
public:
    static constexpr float64_t kLargeCost = 1.0e9;

    // Stride-flexible view so callers can pass a topLeftCorner() block of a
    // fixed-size matrix (or a plain MatrixXd) without copying (A3.2).
    using CostMatrixRef =
        Eigen::Ref<const Eigen::MatrixXd, 0, Eigen::OuterStride<>>;

    HungarianSolver() = default;

    // Solve the rectangular assignment problem N tracks x M detections.
    // assignment[i] = j means track i claims detection j; -1 means unassigned.
    // unassigned_detections collects detection indices no track claimed.
    // A cost entry >= kLargeCost is treated as a gated-out pair and the
    // corresponding assignment is dropped after the square-padded solve.
    bool solve(const CostMatrixRef& cost,
               FixedVector<int32_t, TRACK_MAX_TRACKS>& assignment,
               FixedVector<int32_t, TRACK_MAX_TRACKS>& unassigned_detections);

private:
    // JV is stated in terms of a balanced NxN cost matrix. Both N (tracks) and
    // M (detections) are bounded by TRACK_MAX_TRACKS by contract with the caller,
    // so a single fixed-size pad buffer covers every input dimension.
    static constexpr uint32_t kMaxDim = TRACK_MAX_TRACKS;

    // Working arrays are sized kMaxDim+1 because the JV algorithm references a
    // virtual 0-th source column in the augmenting-path construction.
    std::array<std::array<float64_t, kMaxDim>, kMaxDim> padded_cost_{};
    std::array<float64_t, kMaxDim + 1U> u_{};
    std::array<float64_t, kMaxDim + 1U> v_{};
    std::array<int32_t,   kMaxDim + 1U> p_{};
    std::array<int32_t,   kMaxDim + 1U> way_{};
    std::array<float64_t, kMaxDim + 1U> minv_{};
    std::array<bool,      kMaxDim + 1U> used_{};
};

} // namespace cuas
