// @file position_update.hpp
// @brief Shared position-measurement update and likelihood for the CV/CA/CT models.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace cuas {
namespace estimation {

static constexpr float64_t kLikelihoodFloor = 1e-12;

// All three motion models observe position only, H = [I3 | 0], so the
// innovation covariance is the position block of P plus R. Cholesky
// replaces S.inverse(): when S is not positive definite (info() !=
// Success) the covariance has already lost PSD and a gain computed from
// it is garbage, so the update is skipped and the caller keeps the
// prediction (audit B3). Joseph form keeps P symmetric PSD under rounding
// where (I - KH) P does not.
template <int N>
bool positionUpdate(Eigen::Matrix<float64_t, N, 1>& x,
                    Eigen::Matrix<float64_t, N, N>& P,
                    const Eigen::Vector3d& z,
                    const Eigen::Matrix3d& R)
{
    const Eigen::Vector3d y = z - x.template head<3>();
    const Eigen::Matrix3d S = P.template topLeftCorner<3, 3>() + R;
    const Eigen::LLT<Eigen::Matrix3d> llt(S);
    if (llt.info() != Eigen::Success) {
        return false;
    }
    // K = P H^T S^-1; S is symmetric, so K^T = S^-1 (P H^T)^T.
    const Eigen::Matrix<float64_t, N, 3> PHt = P.template leftCols<3>();
    const Eigen::Matrix<float64_t, N, 3> K   = llt.solve(PHt.transpose()).transpose();
    x += K * y;

    Eigen::Matrix<float64_t, N, N> IKH = Eigen::Matrix<float64_t, N, N>::Identity();
    IKH.template leftCols<3>() -= K;
    P = (IKH * P * IKH.transpose()) + (K * R * K.transpose());
    P = 0.5 * (P + P.transpose());
    return true;
}

template <int N>
float64_t positionLikelihood(const Eigen::Matrix<float64_t, N, 1>& x,
                             const Eigen::Matrix<float64_t, N, N>& P,
                             const Eigen::Vector3d& z,
                             const Eigen::Matrix3d& R)
{
    const Eigen::Vector3d y = z - x.template head<3>();
    const Eigen::Matrix3d S = P.template topLeftCorner<3, 3>() + R;
    const Eigen::LLT<Eigen::Matrix3d> llt(S);
    if (llt.info() != Eigen::Success) {
        return kLikelihoodFloor;
    }
    // det(S) = (prod diag L)^2; the old code floored det at 1e-12.
    const Eigen::Matrix3d L = llt.matrixL();
    const float64_t sqrt_det = L(0, 0) * L(1, 1) * L(2, 2);
    if (!(sqrt_det > 1e-6)) {
        return kLikelihoodFloor;
    }
    const float64_t maha = y.dot(llt.solve(y));
    const float64_t norm = std::pow(2.0 * M_PI, 1.5) * sqrt_det;
    const float64_t like = std::exp(-0.5 * maha) / norm;
    // NaN/inf takes the floor rather than poisoning the IMM weights.
    if (!std::isfinite(like)) {
        return kLikelihoodFloor;
    }
    return std::max(like, kLikelihoodFloor);
}

template <int N>
void symmetrize(Eigen::Matrix<float64_t, N, N>& P)
{
    P = 0.5 * (P + P.transpose());
}

} // namespace estimation
} // namespace cuas
