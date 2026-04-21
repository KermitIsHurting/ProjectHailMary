// @file kinematic_predictor.hpp
// @brief Forward-propagation trajectory predictor over a fixed horizon.
#pragma once

#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

static constexpr uint32_t kMaxTrajectorySteps = 64U;

class KinematicPredictor {
public:
    struct TrajectoryResult {
        FixedVector<Eigen::Vector3d, kMaxTrajectorySteps> positions;
        FixedVector<float64_t, kMaxTrajectorySteps>       timestamps_sec;
        FixedVector<float64_t, kMaxTrajectorySteps>       uncertainty_radii_m;
        FixedVector<float64_t, kMaxTrajectorySteps>       bearing_deg;
        FixedVector<float64_t, kMaxTrajectorySteps>       elevation_deg;
        float64_t final_bearing_deg   = 0.0;
        float64_t final_elevation_deg = 0.0;
    };

    KinematicPredictor() = default;

    TrajectoryResult propagateForward(
        const Eigen::VectorXd& state,
        const Eigen::MatrixXd& covariance,
        const std::array<float64_t, 3>& model_weights,
        const Eigen::MatrixXd& F_blended,
        const Eigen::MatrixXd& Q_blended,
        float64_t step_dt,
        int32_t n_steps);

    static Eigen::VectorXd predictCvStep(const Eigen::VectorXd& state, float64_t dt);
    static Eigen::VectorXd predictCaStep(const Eigen::VectorXd& state, float64_t dt);
    static Eigen::VectorXd predictCtStep(const Eigen::VectorXd& state, float64_t dt);

    static Eigen::VectorXd build_state_from_position_speed(
        float64_t x_m, float64_t y_m, float64_t z_m, float64_t speed_mps);
    static Eigen::MatrixXd build_initial_covariance_6d();
    static Eigen::MatrixXd build_transition_matrix_6d(float64_t step_dt);
    static Eigen::MatrixXd build_process_noise_6d(float64_t step_dt,
                                                  float64_t sigma_a_sq);
};

} // namespace cuas
