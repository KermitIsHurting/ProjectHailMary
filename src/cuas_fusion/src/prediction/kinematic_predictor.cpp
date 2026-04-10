#include "cuas_fusion/prediction/kinematic_predictor.hpp"

#include <cmath>

namespace cuas {

KinematicPredictor::TrajectoryResult KinematicPredictor::propagateForward(
    const Eigen::VectorXd& state,
    const Eigen::MatrixXd& covariance,
    const std::array<double, 3>& /*model_weights*/,
    const Eigen::MatrixXd& F_blended,
    const Eigen::MatrixXd& Q_blended,
    double step_dt,
    int n_steps)
{
    TrajectoryResult result;
    result.positions.reserve(n_steps);
    result.timestamps_sec.reserve(n_steps);
    result.uncertainty_radii_m.reserve(n_steps);
    result.bearing_deg.reserve(n_steps);
    result.elevation_deg.reserve(n_steps);

    Eigen::VectorXd x_cur = state.head(6);
    Eigen::MatrixXd P_cur = covariance.topLeftCorner(6, 6);
    Eigen::MatrixXd F6 = F_blended.topLeftCorner(6, 6);
    Eigen::MatrixXd Q6 = Q_blended.topLeftCorner(6, 6);

    double t = 0.0;

    for (int i = 0; i < n_steps; ++i) {
        t += step_dt;
        x_cur = F6 * x_cur;
        P_cur = F6 * P_cur * F6.transpose() + Q6;

        Eigen::Vector3d pos = x_cur.head(3);
        double unc = std::sqrt(P_cur.block<3, 3>(0, 0).trace());
        double b = std::atan2(pos.y(), pos.x()) * 180.0 / M_PI;
        double xy = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
        double e = std::atan2(pos.z(), xy) * 180.0 / M_PI;

        result.positions.push_back(pos);
        result.timestamps_sec.push_back(t);
        result.uncertainty_radii_m.push_back(unc);
        result.bearing_deg.push_back(b);
        result.elevation_deg.push_back(e);
    }

    if (!result.positions.empty()) {
        result.final_bearing_deg = result.bearing_deg.back();
        result.final_elevation_deg = result.elevation_deg.back();
    }

    return result;
}

} // namespace cuas
