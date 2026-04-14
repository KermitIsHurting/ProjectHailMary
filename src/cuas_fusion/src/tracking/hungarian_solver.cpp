// @file hungarian_solver.cpp
// @brief Jonker-Volgenant O(n^3) implementation over a square-padded buffer.
#include "cuas_fusion/tracking/hungarian_solver.hpp"

#include <algorithm>
#include <limits>

namespace cuas {

bool HungarianSolver::solve(const Eigen::MatrixXd& cost,
                            FixedVector<int32_t, TRACK_MAX_TRACKS>& assignment,
                            FixedVector<int32_t, TRACK_MAX_TRACKS>& unassigned_detections)
{
    assignment.clear();
    unassigned_detections.clear();

    const uint32_t rows = static_cast<uint32_t>(cost.rows());
    const uint32_t cols = static_cast<uint32_t>(cost.cols());

    // Caller bounds both dimensions by TRACK_MAX_TRACKS; reject rather than
    // silently truncate so upstream saturation stays visible.
    if ((rows > kMaxDim) || (cols > kMaxDim)) {
        return false;
    }

    for (uint32_t i = 0U; i < rows; ++i) {
        (void)assignment.push_back(-1);
    }

    if ((rows == 0U) || (cols == 0U)) {
        for (uint32_t j = 0U; j < cols; ++j) {
            (void)unassigned_detections.push_back(static_cast<int32_t>(j));
        }
        return true;
    }

    // JV requires a balanced cost matrix; pad the shorter side with kLargeCost
    // so any synthetic pair is priced out and will be filtered after the solve.
    const uint32_t k = std::max(rows, cols);
    for (uint32_t i = 0U; i < k; ++i) {
        for (uint32_t j = 0U; j < k; ++j) {
            if ((i < rows) && (j < cols)) {
                padded_cost_[i][j] = cost(static_cast<Eigen::Index>(i),
                                          static_cast<Eigen::Index>(j));
            } else {
                padded_cost_[i][j] = kLargeCost;
            }
        }
    }

    for (uint32_t idx = 0U; idx <= k; ++idx) {
        u_[idx]   = 0.0;
        v_[idx]   = 0.0;
        p_[idx]   = 0;
        way_[idx] = 0;
    }

    const float64_t kInf = std::numeric_limits<float64_t>::max();

    for (uint32_t i = 1U; i <= k; ++i) {
        p_[0U] = static_cast<int32_t>(i);
        int32_t j0 = 0;

        for (uint32_t idx = 0U; idx <= k; ++idx) {
            minv_[idx] = kInf;
            used_[idx] = false;
        }

        do {
            used_[static_cast<uint32_t>(j0)] = true;
            const int32_t   i0    = p_[static_cast<uint32_t>(j0)];
            float64_t       delta = kInf;
            int32_t         j1    = 0;

            for (uint32_t j = 1U; j <= k; ++j) {
                if (!used_[j]) {
                    const float64_t cur =
                        padded_cost_[static_cast<uint32_t>(i0 - 1)][j - 1U]
                        - u_[static_cast<uint32_t>(i0)]
                        - v_[j];
                    if (cur < minv_[j]) {
                        minv_[j] = cur;
                        way_[j]  = j0;
                    }
                    if (minv_[j] < delta) {
                        delta = minv_[j];
                        j1    = static_cast<int32_t>(j);
                    }
                }
            }

            for (uint32_t j = 0U; j <= k; ++j) {
                if (used_[j]) {
                    u_[static_cast<uint32_t>(p_[j])] += delta;
                    v_[j]                            -= delta;
                } else {
                    minv_[j] -= delta;
                }
            }

            j0 = j1;
        } while (p_[static_cast<uint32_t>(j0)] != 0);

        do {
            const int32_t j1 = way_[static_cast<uint32_t>(j0)];
            p_[static_cast<uint32_t>(j0)] = p_[static_cast<uint32_t>(j1)];
            j0 = j1;
        } while (j0 != 0);
    }

    // p_[j] = i means 1-indexed column j was assigned to 1-indexed row i.
    // Translate to 0-indexed and drop any pair priced at kLargeCost (either a
    // gated real pair or a synthetic pad slot).
    std::array<bool, kMaxDim> det_claimed{};
    for (uint32_t j = 1U; j <= k; ++j) {
        const int32_t i = p_[j];
        if (i <= 0) {
            continue;
        }
        const uint32_t track_idx = static_cast<uint32_t>(i - 1);
        const uint32_t det_idx   = j - 1U;
        if ((track_idx < rows) && (det_idx < cols)) {
            if (padded_cost_[track_idx][det_idx] < kLargeCost) {
                assignment[track_idx] = static_cast<int32_t>(det_idx);
                det_claimed[det_idx]  = true;
            }
        }
    }

    for (uint32_t j = 0U; j < cols; ++j) {
        if (!det_claimed[j]) {
            (void)unassigned_detections.push_back(static_cast<int32_t>(j));
        }
    }

    return true;
}

} // namespace cuas
