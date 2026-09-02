// @file measurement_models.cpp
// @brief Radar (pos+Doppler) and camera (bearing-only) measurement models.
#include "cuas_fusion/tracking/measurement_models.hpp"

#include "cuas_fusion/common/constants.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace cuas {

bool RadarMeasurementModel::predict(const Vector6d& x, MeasVector& z_pred)
{
    if (!x.allFinite()) {
        return false;
    }
    const Eigen::Vector3d p = x.head<3>();
    const Eigen::Vector3d v = x.tail<3>();
    const float64_t range = p.norm();
    if (!(range > kRadarMinRangeM)) {
        return false;
    }
    z_pred.head<3>() = p;
    z_pred(3) = p.dot(v) / range;
    return true;
}

bool RadarMeasurementModel::jacobian(const Vector6d& x, MeasJacobian& H)
{
    if (!x.allFinite()) {
        return false;
    }
    const Eigen::Vector3d p = x.head<3>();
    const Eigen::Vector3d v = x.tail<3>();
    const float64_t range = p.norm();
    if (!(range > kRadarMinRangeM)) {
        return false;
    }
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    // d(r_dot)/dp = v/r - p(p·v)/r^3 = (v - r_dot*(p/r))/r ; d(r_dot)/dv = p/r
    const float64_t rdot = p.dot(v) / range;
    const Eigen::Vector3d dp = (v - ((rdot / range) * p)) / range;
    const Eigen::Vector3d dv = p / range;
    H.block<1, 3>(3, 0) = dp.transpose();
    H.block<1, 3>(3, 3) = dv.transpose();
    return true;
}

RadarMeasurementModel::MeasMatrix RadarMeasurementModel::noise()
{
    MeasMatrix R = MeasMatrix::Zero();
    const float64_t pos_var = kRadarDetectionSigmaM * kRadarDetectionSigmaM;
    R(0, 0) = pos_var;
    R(1, 1) = pos_var;
    R(2, 2) = pos_var;
    R(3, 3) = kRadarDopplerSigmaMps * kRadarDopplerSigmaMps;
    return R;
}

bool CameraBearingModel::init(const ExtrinsicTransform& extrinsic,
                              const float64_t fx, const float64_t fy,
                              const float64_t cx, const float64_t cy)
{
    initialized_ = false;
    std::array<float32_t, 9> rot{};
    if (!extrinsicRotationMatrix(extrinsic, rot)) {
        return false;
    }
    if (!std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy) ||
        !(fx > 0.0) || !(fy > 0.0)) {
        return false;
    }
    for (std::size_t r = 0U; r < 3U; ++r) {
        for (std::size_t c = 0U; c < 3U; ++c) {
            R_(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
                static_cast<float64_t>(rot[(r * 3U) + c]);
        }
    }
    t_ = Eigen::Vector3d(static_cast<float64_t>(extrinsic.t_x_m),
                         static_cast<float64_t>(extrinsic.t_y_m),
                         static_cast<float64_t>(extrinsic.t_z_m));
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    initialized_ = true;
    return true;
}

bool CameraBearingModel::pixelToBearing(const float32_t u, const float32_t v,
                                        MeasVector& z) const
{
    if (!initialized_ || !std::isfinite(u) || !std::isfinite(v)) {
        return false;
    }
    const float64_t rx = (static_cast<float64_t>(u) - cx_) / fx_;
    const float64_t ry = (static_cast<float64_t>(v) - cy_) / fy_;
    z(0) = std::atan2(rx, 1.0);
    z(1) = std::atan2(-ry, std::sqrt((rx * rx) + 1.0));
    return true;
}

bool CameraBearingModel::predict(const Vector6d& x, MeasVector& z_pred) const
{
    if (!initialized_ || !x.allFinite()) {
        return false;
    }
    const Eigen::Vector3d p_cam = (R_ * x.head<3>()) + t_;
    if (!(p_cam(2) > kCameraMinDepthM)) {
        return false;
    }
    const float64_t rho =
        std::sqrt((p_cam(0) * p_cam(0)) + (p_cam(2) * p_cam(2)));
    z_pred(0) = std::atan2(p_cam(0), p_cam(2));
    z_pred(1) = std::atan2(-p_cam(1), rho);
    return true;
}

bool CameraBearingModel::jacobian(const Vector6d& x, MeasJacobian& H) const
{
    if (!initialized_ || !x.allFinite()) {
        return false;
    }
    const Eigen::Vector3d p_cam = (R_ * x.head<3>()) + t_;
    const float64_t xc = p_cam(0);
    const float64_t yc = p_cam(1);
    const float64_t zc = p_cam(2);
    if (!(zc > kCameraMinDepthM)) {
        return false;
    }
    // rho >= zc > kCameraMinDepthM bounds every division below.
    const float64_t rho2 = (xc * xc) + (zc * zc);
    const float64_t r2 = rho2 + (yc * yc);
    const float64_t rho = std::sqrt(rho2);
    Eigen::Matrix<float64_t, 2, 3> G;
    G(0, 0) = zc / rho2;
    G(0, 1) = 0.0;
    G(0, 2) = -xc / rho2;
    G(1, 0) = (xc * yc) / (r2 * rho);
    G(1, 1) = -rho / r2;
    G(1, 2) = (zc * yc) / (r2 * rho);
    H.setZero();
    H.block<2, 3>(0, 0) = G * R_;
    return true;
}

CameraBearingModel::MeasMatrix CameraBearingModel::noise() const
{
    MeasMatrix R = MeasMatrix::Zero();
    // Guarded so an uninitialized model yields a finite (if meaningless) R.
    const float64_t s_az = kCameraPixelSigmaPx / ((fx_ > 0.0) ? fx_ : 1.0);
    const float64_t s_el = kCameraPixelSigmaPx / ((fy_ > 0.0) ? fy_ : 1.0);
    R(0, 0) = s_az * s_az;
    R(1, 1) = s_el * s_el;
    return R;
}

} // namespace cuas
