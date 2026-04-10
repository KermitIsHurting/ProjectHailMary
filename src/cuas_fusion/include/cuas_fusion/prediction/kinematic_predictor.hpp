#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace cuas {

/// Forward trajectory propagation using IMM-blended dynamics
class KinematicPredictor {
public:
    struct TrajectoryResult {
        std::vector<Eigen::Vector3d> positions;
        std::vector<double> timestamps_sec;
        std::vector<double> uncertainty_radii_m;
        std::vector<double> bearing_deg;
        std::vector<double> elevation_deg;
        double final_bearing_deg = 0.0;
        double final_elevation_deg = 0.0;
    };

    KinematicPredictor() = default;

    /// Propagate state forward n_steps at step_dt intervals
    TrajectoryResult propagateForward(
        const Eigen::VectorXd& state,
        const Eigen::MatrixXd& covariance,
        const std::array<double, 3>& model_weights,
        const Eigen::MatrixXd& F_blended,
        const Eigen::MatrixXd& Q_blended,
        double step_dt,
        int n_steps);
};

} // namespace cuas
