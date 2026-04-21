// @file kinematic_predictor.cpp
// @brief Forward-propagation trajectory prediction with IMM model blending.
#include "cuas_fusion/prediction/kinematic_predictor.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

KinematicPredictor::TrajectoryResult KinematicPredictor::propagateForward(
    const Eigen::VectorXd& state,
    const Eigen::MatrixXd& covariance,
    const std::array<float64_t, 3>& model_weights,
    const Eigen::MatrixXd& F_blended,
    const Eigen::MatrixXd& Q_blended,
    float64_t step_dt,
    int32_t n_steps)
{
    TrajectoryResult result;

    Eigen::VectorXd x_cv = state.head(6);
    Eigen::VectorXd x_ca = state.head(6);
    Eigen::VectorXd x_ct = state.head(6);
    Eigen::MatrixXd P_cur = covariance.topLeftCorner(6, 6);
    const Eigen::MatrixXd F6 = F_blended.topLeftCorner(6, 6);
    const Eigen::MatrixXd Q6 = Q_blended.topLeftCorner(6, 6);

    float64_t t = 0.0;

    for (int32_t i = 0; i < n_steps; ++i) {
        t += step_dt;

        x_cv = predictCvStep(x_cv, step_dt);
        x_ca = predictCaStep(x_ca, step_dt);
        x_ct = predictCtStep(x_ct, step_dt);

        const Eigen::Vector3d blended_pos =
            model_weights[0] * x_cv.head<3>() +
            model_weights[1] * x_ca.head<3>() +
            model_weights[2] * x_ct.head<3>();

        const Eigen::Vector3d blended_vel =
            model_weights[0] * x_cv.segment<3>(3) +
            model_weights[1] * x_ca.segment<3>(3) +
            model_weights[2] * x_ct.segment<3>(3);

        (void)blended_vel;

        P_cur = F6 * P_cur * F6.transpose() + Q6;

        // WHY: radius is the RMS per-axis position stddev, not the sum of
        // variances — trace(P_pos) is sum of three variances, so divide by 3
        // then take the root. Capped at kMaxUncertaintyRadiusM because the
        // overlay cannot render arcs larger than the scene.
        const float64_t trace_pos = P_cur.block<3, 3>(0, 0).trace();
        const float64_t rms       = std::sqrt(trace_pos / 3.0);
        const float64_t unc       = std::min(rms,
            static_cast<float64_t>(cuas::kMaxUncertaintyRadiusM));
        const float64_t b   = std::atan2(blended_pos.y(), blended_pos.x()) * 180.0 / M_PI;
        const float64_t xy  = std::sqrt(blended_pos.x() * blended_pos.x() +
                                        blended_pos.y() * blended_pos.y());
        const float64_t e   = std::atan2(blended_pos.z(), xy) * 180.0 / M_PI;

        (void)result.positions.push_back(blended_pos);
        (void)result.timestamps_sec.push_back(t);
        (void)result.uncertainty_radii_m.push_back(unc);
        (void)result.bearing_deg.push_back(b);
        (void)result.elevation_deg.push_back(e);
    }

    if (!result.positions.empty()) {
        result.final_bearing_deg   = result.bearing_deg.back();
        result.final_elevation_deg = result.elevation_deg.back();
    }

    return result;
}

Eigen::VectorXd KinematicPredictor::predictCvStep(
    const Eigen::VectorXd& state, float64_t dt)
{
    Eigen::VectorXd x = state;
    x(0) += state(3) * dt;
    x(1) += state(4) * dt;
    x(2) += state(5) * dt;
    return x;
}

Eigen::VectorXd KinematicPredictor::predictCaStep(
    const Eigen::VectorXd& state, float64_t dt)
{
    Eigen::VectorXd x = state;
    x(0) += state(3) * dt;
    x(1) += state(4) * dt;
    x(2) += state(5) * dt;
    return x;
}

Eigen::VectorXd KinematicPredictor::predictCtStep(
    const Eigen::VectorXd& state, float64_t dt)
{
    Eigen::VectorXd x = state;
    x(0) += state(3) * dt;
    x(1) += state(4) * dt;
    x(2) += state(5) * dt;
    return x;
}

Eigen::VectorXd KinematicPredictor::build_state_from_position_speed(
    float64_t x_m, float64_t y_m, float64_t z_m, float64_t speed_mps)
{
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
    state(0) = x_m;
    state(1) = y_m;
    state(2) = z_m;
    if (speed_mps > 0.0) {
        const float64_t bearing = std::atan2(y_m, x_m);
        state(3) = speed_mps * std::cos(bearing);
        state(4) = speed_mps * std::sin(bearing);
    }
    return state;
}

Eigen::MatrixXd KinematicPredictor::build_initial_covariance_6d()
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6, 6);
    P.diagonal() << 1.0, 1.0, 1.0, 0.25, 0.25, 0.25;
    return P;
}

Eigen::MatrixXd KinematicPredictor::build_transition_matrix_6d(float64_t step_dt)
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = step_dt;
    F(1, 4) = step_dt;
    F(2, 5) = step_dt;
    return F;
}

Eigen::MatrixXd KinematicPredictor::build_process_noise_6d(float64_t step_dt,
                                                           float64_t sigma_a_sq)
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    const float64_t dt2 = step_dt * step_dt;
    for (uint32_t i = 0U; i < 3U; ++i) {
        const int32_t ii = static_cast<int32_t>(i);
        Q(ii, ii)         = 0.25 * dt2 * dt2 * sigma_a_sq;
        Q(ii, ii + 3)     = 0.5  * dt2 * step_dt * sigma_a_sq;
        Q(ii + 3, ii)     = 0.5  * dt2 * step_dt * sigma_a_sq;
        Q(ii + 3, ii + 3) = dt2 * sigma_a_sq;
    }
    return Q;
}

} // namespace cuas
