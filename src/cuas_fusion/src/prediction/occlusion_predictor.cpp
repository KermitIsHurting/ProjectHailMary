#include "cuas_fusion/prediction/occlusion_predictor.hpp"

#include <cmath>

namespace cuas {

void OcclusionPredictor::configure(double max_occlusion_sec, double mahalanobis_gate)
{
    max_occlusion_sec_ = max_occlusion_sec;
    mahalanobis_gate_ = mahalanobis_gate;
}

KinematicPredictor::TrajectoryResult OcclusionPredictor::propagateGhost(
    GhostTrack& ghost,
    double current_time,
    double step_dt,
    int n_steps,
    const Eigen::MatrixXd& F_blended,
    const Eigen::MatrixXd& Q_blended)
{
    double dt_since = current_time - ghost.occlusion_start_time_sec;
    if (dt_since > max_occlusion_sec_) {
        ghost.expired = true;
        return {};
    }

    Eigen::MatrixXd P_inflated = ghost.covariance;
    double factor = 1.0 + 0.1 * dt_since;
    P_inflated.diagonal() *= factor;
    // Cap each diagonal element to prevent unbounded growth
    for (int i = 0; i < P_inflated.rows(); ++i) {
        if (P_inflated(i, i) > 2.0) P_inflated(i, i) = 2.0;
    }

    return propagator_.propagateForward(
        ghost.state, P_inflated, ghost.model_weights,
        F_blended, Q_blended, step_dt, n_steps);
}

bool OcclusionPredictor::reacquire(
    const GhostTrack& ghost,
    double meas_x, double meas_y, double meas_z)
{
    Eigen::Vector3d predicted = ghost.state.head(3);
    Eigen::Vector3d innovation(meas_x - predicted(0),
                                meas_y - predicted(1),
                                meas_z - predicted(2));

    Eigen::Matrix3d R_default = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = ghost.covariance.block<3, 3>(0, 0) + R_default;
    double mahal = std::sqrt((innovation.transpose() * S.inverse() * innovation)(0, 0));
    return mahal < mahalanobis_gate_;
}

} // namespace cuas
