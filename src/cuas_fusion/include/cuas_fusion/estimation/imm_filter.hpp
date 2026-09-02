// @file imm_filter.hpp
// @brief Interacting multiple-model filter combining CV/CA/CT models.
#pragma once

#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/estimation/kalman_cv.hpp"
#include "cuas_fusion/estimation/kalman_ca.hpp"
#include "cuas_fusion/estimation/kalman_ct.hpp"

#include <Eigen/Dense>
#include <array>

namespace cuas {

class ImmFilter {
public:
    ImmFilter();

    void init(const Vector6d& x0, const Matrix6d& P0);
    void predict(float64_t dt);
    void update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);
    Vector6d getState() const;
    Matrix6d getCovariance() const;
    std::array<float64_t, 3> getModelWeights() const;
    Matrix6d getMixedF(float64_t dt) const;
    Matrix6d getMixedQ(float64_t dt) const;
    void setModelNoise(uint32_t model_index, float64_t sigma);
    void setVelocity(const Eigen::Vector3d& v);
    Vector6d getModelState(uint32_t index) const;
    Matrix6d getCvTransitionMatrix(float64_t dt) const;
    Matrix6d getCaTransitionMatrix(float64_t dt) const;
    Matrix6d getCtTransitionMatrix(float64_t dt) const;

private:
    // c_bar_j = sum_i p_ij mu_i: the mixed prior shared by predict() and update().
    std::array<float64_t, 3> mixedPrior() const;

    // Every mode stays reachable: a weight this small still recovers within
    // a few scans once its likelihood dominates (RC-7).
    static constexpr float64_t kMuFloor = 1e-6;

    KalmanCV cv_;
    KalmanCA ca_;
    KalmanCT ct_;

    std::array<float64_t, 3> mu_{};
    std::array<std::array<float64_t, 3>, 3> tp_{};

    bool initialized_ = false;
};

} // namespace cuas
