// @file kinematic_predictor.cpp
// @brief Forward-propagation trajectory prediction.
#include "cuas_fusion/prediction/kinematic_predictor.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <cmath>

namespace cuas {

KinematicPredictor::TrajectoryResult KinematicPredictor::propagateForward(
    const Eigen::VectorXd& state,
    const Eigen::MatrixXd& covariance,
    const std::array<float64_t, 3>& /*model_weights*/,
    const Eigen::MatrixXd& F_blended,
    const Eigen::MatrixXd& Q_blended,
    float64_t step_dt,
    int32_t n_steps)
{
    TrajectoryResult result;
    const std::size_t reserve_n = (n_steps > 0) ? static_cast<std::size_t>(n_steps) : 0U;
    result.positions.reserve(reserve_n);
    result.timestamps_sec.reserve(reserve_n);
    result.uncertainty_radii_m.reserve(reserve_n);
    result.bearing_deg.reserve(reserve_n);
    result.elevation_deg.reserve(reserve_n);

    Eigen::VectorXd x_cur = state.head(6);
    Eigen::MatrixXd P_cur = covariance.topLeftCorner(6, 6);
    const Eigen::MatrixXd F6 = F_blended.topLeftCorner(6, 6);
    const Eigen::MatrixXd Q6 = Q_blended.topLeftCorner(6, 6);

    float64_t t = 0.0;

    for (int32_t i = 0; i < n_steps; ++i) {
        t += step_dt;
        x_cur = F6 * x_cur;
        P_cur = F6 * P_cur * F6.transpose() + Q6;

        const Eigen::Vector3d pos = x_cur.head(3);
        const float64_t unc = std::sqrt(P_cur.block<3, 3>(0, 0).trace());
        const float64_t b   = std::atan2(pos.y(), pos.x()) * 180.0 / M_PI;
        const float64_t xy  = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
        const float64_t e   = std::atan2(pos.z(), xy) * 180.0 / M_PI;

        result.positions.push_back(pos);
        result.timestamps_sec.push_back(t);
        result.uncertainty_radii_m.push_back(unc);
        result.bearing_deg.push_back(b);
        result.elevation_deg.push_back(e);
    }

    if (!result.positions.empty()) {
        result.final_bearing_deg   = result.bearing_deg.back();
        result.final_elevation_deg = result.elevation_deg.back();
    }

    return result;
}

} // namespace cuas
