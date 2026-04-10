#pragma once

#include "cuas_fusion/estimation/kalman_cv.hpp"
#include "cuas_fusion/estimation/kalman_ca.hpp"
#include "cuas_fusion/estimation/kalman_ct.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

/// Interacting Multiple Model filter blending CV, CA, CT sub-models
class ImmFilter {
public:
    ImmFilter();

    /// Initialize all sub-models with a 6-DOF state and covariance
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    /// IMM predict cycle
    void predict(double dt);

    /// IMM update cycle with position measurement
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /// Probability-weighted combined state (6x1)
    Eigen::VectorXd getState() const;

    /// Probability-weighted combined covariance (6x6)
    Eigen::MatrixXd getCovariance() const;

    /// Current model probabilities [CV, CA, CT]
    std::array<double, 3> getModelWeights() const;

    /// Weighted blended F matrix for predictor use (6x6)
    Eigen::MatrixXd getMixedF(double dt) const;

    /// Weighted blended Q matrix for predictor use (6x6)
    Eigen::MatrixXd getMixedQ(double dt) const;

private:
    KalmanCV cv_;
    KalmanCA ca_;
    KalmanCT ct_;

    std::array<double, 3> mu_;
    std::array<std::array<double, 3>, 3> tp_;

    bool initialized_ = false;
};

} // namespace cuas
