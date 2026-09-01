// @file kinematic_predictor.hpp
// @brief Constant-velocity forward-propagation trajectory predictor.
//
// CV-only by design: the 6-DOF pos/vel state built from Track.msg carries no
// acceleration or turn-rate information, so CA/CT forward models would have
// nothing to integrate. Publishers report model_weight_cv=1 accordingly.
#pragma once

#include "cuas_fusion/common/eigen_types.hpp"
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
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& Q,
        float64_t step_dt,
        int32_t n_steps);

    static Vector6d predictCvStep(const Vector6d& state, float64_t dt);

    // Takes the track's velocity vector as-is. The previous builder took a
    // speed and pointed it along the position bearing, so every forecast
    // ran radially outward regardless of true heading (same defect as
    // reachability A1.3).
    static Eigen::VectorXd build_state_from_position_velocity(
        float64_t x_m, float64_t y_m, float64_t z_m,
        float64_t vx_mps, float64_t vy_mps, float64_t vz_mps);
    static Eigen::MatrixXd build_initial_covariance_6d();
    static Eigen::MatrixXd build_transition_matrix_6d(float64_t step_dt);
    static Eigen::MatrixXd build_process_noise_6d(float64_t step_dt,
                                                  float64_t sigma_a_sq);
};

} // namespace cuas
