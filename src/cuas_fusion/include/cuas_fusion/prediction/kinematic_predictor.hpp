// @file kinematic_predictor.hpp
// @brief Forward-propagation trajectory predictor over a fixed horizon.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace cuas {

class KinematicPredictor {
public:
    struct TrajectoryResult {
        std::vector<Eigen::Vector3d> positions;
        std::vector<float64_t>       timestamps_sec;
        std::vector<float64_t>       uncertainty_radii_m;
        std::vector<float64_t>       bearing_deg;
        std::vector<float64_t>       elevation_deg;
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
};

} // namespace cuas
