// @file occlusion_predictor.cpp
// @brief Ghost-track propagation through occluded intervals.
#include "cuas_fusion/prediction/occlusion_predictor.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <cmath>

namespace cuas {

void OcclusionPredictor::configure(float64_t max_occlusion_sec, float64_t mahalanobis_gate)
{
    max_occlusion_sec_ = max_occlusion_sec;
    mahalanobis_gate_  = mahalanobis_gate;
}

KinematicPredictor::TrajectoryResult OcclusionPredictor::propagateGhost(
    GhostTrack& ghost,
    float64_t current_time,
    float64_t step_dt,
    int32_t n_steps,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& Q)
{
    const float64_t dt_since = current_time - ghost.occlusion_start_time_sec;
    if (dt_since > max_occlusion_sec_) {
        ghost.expired = true;
        return {};
    }

    Eigen::MatrixXd P_inflated = ghost.covariance;
    const float64_t factor = 1.0 + 0.1 * dt_since;
    P_inflated.diagonal() *= factor;
    // Cap diagonal to prevent unbounded growth during long occlusions
    for (int32_t i = 0; i < static_cast<int32_t>(P_inflated.rows()); ++i) {
        if (P_inflated(i, i) > 2.0) {
            P_inflated(i, i) = 2.0;
        }
    }

    return propagator_.propagateForward(
        ghost.state, P_inflated, F, Q, step_dt, n_steps);
}

bool OcclusionPredictor::reacquire(
    const GhostTrack& ghost,
    float64_t meas_x, float64_t meas_y, float64_t meas_z) const
{
    const Eigen::Vector3d predicted = ghost.state.head(3);
    const Eigen::Vector3d innovation(
        meas_x - predicted(0),
        meas_y - predicted(1),
        meas_z - predicted(2));

    const Eigen::Matrix3d R_default = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d S = ghost.covariance.block<3, 3>(0, 0) + R_default;
    const float64_t mahal = std::sqrt((innovation.transpose() * S.inverse() * innovation)(0, 0));
    return mahal < mahalanobis_gate_;
}

} // namespace cuas
