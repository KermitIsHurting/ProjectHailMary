// @file measurement_models.hpp
// @brief Per-sensor measurement models for the central tracker (P3.2).
//
// Both models express a sensor observation over the shared 6-state
// [position, velocity] block in base_link (base_link -> radar_frame is the
// identity static transform, ICD §7) as the (h(x), H, R) triple the central
// tracker (P3.3+) needs for gating and EKF updates. Nothing here allocates.
//
// Conventions:
//  - Radar z = [x, y, z, r_dot], r_dot = d|p|/dt = (p·v)/|p|: positive is
//    receding. Ingest must reconcile the sensor's Doppler sign once, at
//    the ROS boundary.
//  - Camera z = [azimuth, elevation] of the target ray in the CAMERA frame
//    (pinhole: +z forward, +x right/+u, +y down/+v); positive elevation is
//    up (-y). p_cam = R*p + t with (R, t) from ExtrinsicTransform (P1.1).
#pragma once

#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <Eigen/Dense>

namespace cuas {

// Radar: 3D position plus radial Doppler. Stateless.
class RadarMeasurementModel {
public:
    static constexpr int32_t kMeasDim = 4;
    using MeasVector   = Eigen::Matrix<float64_t, kMeasDim, 1>;
    using MeasMatrix   = Eigen::Matrix<float64_t, kMeasDim, kMeasDim>;
    using MeasJacobian = Eigen::Matrix<float64_t, kMeasDim, 6>;

    // z_pred = [p; (p·v)/|p|]. False (output untouched) for a non-finite
    // state or range below kRadarMinRangeM — NaN takes the reject branch.
    static bool predict(const Vector6d& x, MeasVector& z_pred);

    // H = dh/dx at x; same validity domain as predict().
    static bool jacobian(const Vector6d& x, MeasJacobian& H);

    static MeasMatrix noise();
};

// Camera: bearing-only measurement from a detector pixel through the
// calibrated pinhole and SE(3) extrinsics.
class CameraBearingModel {
public:
    static constexpr int32_t kMeasDim = 2;
    using MeasVector   = Eigen::Matrix<float64_t, kMeasDim, 1>;
    using MeasMatrix   = Eigen::Matrix<float64_t, kMeasDim, kMeasDim>;
    using MeasJacobian = Eigen::Matrix<float64_t, kMeasDim, 6>;

    CameraBearingModel() = default;

    // False for a degenerate quaternion (extrinsicRotationMatrix contract)
    // or non-finite / non-positive intrinsics; the model stays unusable
    // until an init() succeeds.
    bool init(const ExtrinsicTransform& extrinsic,
              float64_t fx, float64_t fy, float64_t cx, float64_t cy);

    // Detector pixel -> bearing measurement. False for non-finite input
    // or an uninitialized model.
    bool pixelToBearing(float32_t u, float32_t v, MeasVector& z) const;

    // h(x): bearing of the state's position. False when uninitialized,
    // the state is non-finite, or the point sits at/behind the image
    // plane (p_cam.z <= kCameraMinDepthM).
    bool predict(const Vector6d& x, MeasVector& z_pred) const;

    // H = dh/dx at x (velocity columns are zero); same domain as predict().
    bool jacobian(const Vector6d& x, MeasJacobian& H) const;

    // Pixel noise mapped through the focal lengths (small-angle).
    MeasMatrix noise() const;

private:
    Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();  // base_link -> camera
    Eigen::Vector3d t_ = Eigen::Vector3d::Zero();
    float64_t fx_ = 0.0;
    float64_t fy_ = 0.0;
    float64_t cx_ = 0.0;
    float64_t cy_ = 0.0;
    bool initialized_ = false;
};

} // namespace cuas
