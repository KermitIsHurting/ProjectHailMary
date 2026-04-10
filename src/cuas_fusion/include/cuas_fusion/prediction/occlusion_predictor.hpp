#pragma once

#include "cuas_fusion/prediction/kinematic_predictor.hpp"

#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace cuas {

/// Maintains ghost tracks for occluded targets and predicts through occlusion
class OcclusionPredictor {
public:
    struct GhostTrack {
        uint32_t track_id = 0;
        Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6);
        std::array<double, 3> model_weights = {0.33, 0.33, 0.34};
        double occlusion_start_time_sec = 0.0;
        bool expired = false;
    };

    OcclusionPredictor() = default;

    /// Set max occlusion duration and Mahalanobis gate
    void configure(double max_occlusion_sec, double mahalanobis_gate);

    /// Propagate a ghost track forward
    KinematicPredictor::TrajectoryResult propagateGhost(
        GhostTrack& ghost,
        double current_time,
        double step_dt,
        int n_steps,
        const Eigen::MatrixXd& F_blended,
        const Eigen::MatrixXd& Q_blended);

    /// Test whether a measurement can reacquire a ghost track
    bool reacquire(const GhostTrack& ghost,
                   double meas_x, double meas_y, double meas_z);

private:
    double max_occlusion_sec_ = 4.0;
    double mahalanobis_gate_ = 3.0;
    KinematicPredictor propagator_;
};

} // namespace cuas
