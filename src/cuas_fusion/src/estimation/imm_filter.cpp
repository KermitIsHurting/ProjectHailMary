// @file imm_filter.cpp
// @brief Interacting multiple-model filter combining CV/CA/CT models.
#include "cuas_fusion/estimation/imm_filter.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <algorithm>
#include <cmath>

namespace cuas {

ImmFilter::ImmFilter()
    : mu_{0.33, 0.33, 0.34}
{
    for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = 0; j < 3; ++j) {
            tp_[i][j] = (i == j) ? 0.90 : 0.05;
        }
    }
}

void ImmFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    cv_.init(x0, P0);
    ca_.init(x0, P0);
    ct_.init(x0, P0);
    mu_ = {0.33, 0.33, 0.34};
    initialized_ = true;
}

void ImmFilter::predict(float64_t dt)
{
    if (!initialized_) {
        return;
    }

    std::array<float64_t, 3> c_bar{};
    for (int32_t j = 0; j < 3; ++j) {
        for (int32_t i = 0; i < 3; ++i) {
            c_bar[j] += tp_[i][j] * mu_[i];
        }
        c_bar[j] = std::max(c_bar[j], 1e-12);
    }

    std::array<std::array<float64_t, 3>, 3> mu_ij{};
    for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = 0; j < 3; ++j) {
            mu_ij[i][j] = tp_[i][j] * mu_[i] / c_bar[j];
        }
    }

    std::array<Eigen::VectorXd, 3> states;
    std::array<Eigen::MatrixXd, 3> covs;
    states[0] = cv_.getState();
    states[1] = ca_.getState();
    states[2] = ct_.getState();
    covs[0] = cv_.getCovariance();
    covs[1] = ca_.getCovariance();
    covs[2] = ct_.getCovariance();

    for (int32_t j = 0; j < 3; ++j) {
        Eigen::VectorXd x0_j = Eigen::VectorXd::Zero(6);
        for (int32_t i = 0; i < 3; ++i) {
            x0_j += mu_ij[i][j] * states[i];
        }

        Eigen::MatrixXd P0_j = Eigen::MatrixXd::Zero(6, 6);
        for (int32_t i = 0; i < 3; ++i) {
            const Eigen::VectorXd dx = states[i] - x0_j;
            P0_j += mu_ij[i][j] * (covs[i] + dx * dx.transpose());
        }

        if (j == 0) {
            cv_.init(x0_j, P0_j);
        } else if (j == 1) {
            ca_.init(x0_j, P0_j);
        } else {
            ct_.init(x0_j, P0_j);
        }
    }

    cv_.predict(dt);
    ca_.predict(dt);
    ct_.predict(dt);
}

void ImmFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
    if (!initialized_) {
        return;
    }

    const float64_t L0 = cv_.likelihood(z, R);
    const float64_t L1 = ca_.likelihood(z, R);
    const float64_t L2 = ct_.likelihood(z, R);

    cv_.update(z, R);
    ca_.update(z, R);
    ct_.update(z, R);

    float64_t c_sum = mu_[0] * L0 + mu_[1] * L1 + mu_[2] * L2;
    if (c_sum < 1e-30) {
        c_sum = 1e-30;
    }

    mu_[0] = mu_[0] * L0 / c_sum;
    mu_[1] = mu_[1] * L1 / c_sum;
    mu_[2] = mu_[2] * L2 / c_sum;

    const float64_t sum = mu_[0] + mu_[1] + mu_[2];
    mu_[0] /= sum;
    mu_[1] /= sum;
    mu_[2] /= sum;
}

Eigen::VectorXd ImmFilter::getState() const
{
    return mu_[0] * cv_.getState() + mu_[1] * ca_.getState() + mu_[2] * ct_.getState();
}

Eigen::MatrixXd ImmFilter::getCovariance() const
{
    const Eigen::VectorXd x_combined = getState();
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6, 6);

    std::array<Eigen::VectorXd, 3> states = {cv_.getState(), ca_.getState(), ct_.getState()};
    std::array<Eigen::MatrixXd, 3> covs   = {cv_.getCovariance(), ca_.getCovariance(), ct_.getCovariance()};

    for (int32_t i = 0; i < 3; ++i) {
        const Eigen::VectorXd dx = states[i] - x_combined;
        P += mu_[i] * (covs[i] + dx * dx.transpose());
    }
    return P;
}

std::array<float64_t, 3> ImmFilter::getModelWeights() const { return mu_; }

void ImmFilter::setModelNoise(uint32_t model_index, float64_t sigma)
{
    switch (model_index) {
        case 0U: {
            cv_.setSigmaA(sigma);
            break;
        }
        case 1U: {
            ca_.setSigmaJ(sigma);
            break;
        }
        case 2U: {
            break;
        }
        default: {
            break;
        }
    }
}

Eigen::MatrixXd ImmFilter::getMixedF(float64_t dt) const
{
    return mu_[0] * cv_.getF(dt) + mu_[1] * ca_.getF(dt) + mu_[2] * ct_.getF(dt);
}

Eigen::MatrixXd ImmFilter::getMixedQ(float64_t dt) const
{
    return mu_[0] * cv_.getQ(dt) + mu_[1] * ca_.getQ(dt) + mu_[2] * ct_.getQ(dt);
}

} // namespace cuas
