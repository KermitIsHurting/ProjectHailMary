// @file imm_filter.hpp
// @brief Interacting multiple-model filter combining CV/CA/CT models.
#pragma once

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

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    void predict(float64_t dt);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    std::array<float64_t, 3> getModelWeights() const;
    Eigen::MatrixXd getMixedF(float64_t dt) const;
    Eigen::MatrixXd getMixedQ(float64_t dt) const;
    void setModelNoise(uint32_t model_index, float64_t sigma);

private:
    KalmanCV cv_;
    KalmanCA ca_;
    KalmanCT ct_;

    std::array<float64_t, 3> mu_{};
    std::array<std::array<float64_t, 3>, 3> tp_{};

    bool initialized_ = false;
};

} // namespace cuas
