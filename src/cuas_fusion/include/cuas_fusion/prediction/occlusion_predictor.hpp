// @file occlusion_predictor.hpp
// @brief Ghost-track predictor that propagates through occlusions.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/prediction/kinematic_predictor.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

class OcclusionPredictor {
public:
    struct GhostTrack {
        uint32_t track_id = 0U;
        Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6);
        std::array<float64_t, 3> model_weights = {0.33, 0.33, 0.34};
        float64_t occlusion_start_time_sec = 0.0;
        bool      expired = false;
    };

    OcclusionPredictor() = default;

    void configure(float64_t max_occlusion_sec, float64_t mahalanobis_gate);

    KinematicPredictor::TrajectoryResult propagateGhost(
        GhostTrack& ghost,
        float64_t current_time,
        float64_t step_dt,
        int32_t n_steps,
        const Eigen::MatrixXd& F_blended,
        const Eigen::MatrixXd& Q_blended);

    bool reacquire(const GhostTrack& ghost,
                   float64_t meas_x, float64_t meas_y, float64_t meas_z) const;

private:
    float64_t max_occlusion_sec_ = 4.0;
    float64_t mahalanobis_gate_  = 3.0;
    KinematicPredictor propagator_;
};

} // namespace cuas
