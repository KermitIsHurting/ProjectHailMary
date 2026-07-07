// @file track_manager.hpp
// @brief Hungarian-assigned, Mahalanobis-gated track manager over a fixed slot array.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/hungarian_solver.hpp"
#include "cuas_fusion/tracking/track.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

class TrackManager {
public:
    TrackManager() = default;

    bool init();

    bool update(const FusedDetection* detections,
                uint32_t det_count,
                FixedVector<Track, TRACK_MAX_TRACKS>& confirmed_out);

private:
    struct TrackEntry {
        Track           track{};
        Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
        int32_t         hit_count  = 0;
        int32_t         miss_count = 0;
        bool            active     = false;
    };

    uint32_t allocateSlot() const;
    void     initiateTrack(uint32_t slot, const FusedDetection& det);
    void     applyDetection(uint32_t slot, const FusedDetection& det);
    void     applyMiss(uint32_t slot);

    // Fixed capacity: N and M are both bounded by TRACK_MAX_TRACKS, so the
    // per-scan resize was pure reallocation churn (A3.2); the solver reads
    // the topLeftCorner(N, M) view.
    using CostMatrix = Eigen::Matrix<float64_t,
                                     static_cast<int32_t>(TRACK_MAX_TRACKS),
                                     static_cast<int32_t>(TRACK_MAX_TRACKS)>;

    std::array<TrackEntry, TRACK_MAX_TRACKS> entries_{};
    uint32_t        next_id_      = 1U;
    HungarianSolver solver_{};
    CostMatrix      cost_matrix_  = CostMatrix::Zero();
    Eigen::Matrix3d R_detection_ = Eigen::Matrix3d::Identity();
};

} // namespace cuas
