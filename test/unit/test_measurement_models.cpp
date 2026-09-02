// test_measurement_models.cpp
// Unit tests for the P3.2 per-sensor measurement models. Jacobians are
// checked against central finite differences of the models' own h(x);
// geometry and frame conventions are pinned by known-answer cases so a
// transposed rotation or flipped sign cannot pass.

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/measurement_models.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using cuas::CameraBearingModel;
using cuas::RadarMeasurementModel;

cuas::Vector6d makeState(cuas::float64_t px, cuas::float64_t py,
                         cuas::float64_t pz, cuas::float64_t vx,
                         cuas::float64_t vy, cuas::float64_t vz)
{
    cuas::Vector6d x;
    x << px, py, pz, vx, vy, vz;
    return x;
}

// --- Radar --------------------------------------------------------------

TEST(RadarMeasurementModel, RadialTargetDopplerIsRangeRate)
{
    const cuas::Vector6d x = makeState(10.0, 0.0, 0.0, -3.0, 0.0, 0.0);
    RadarMeasurementModel::MeasVector z;
    ASSERT_TRUE(RadarMeasurementModel::predict(x, z));
    EXPECT_DOUBLE_EQ(z(0), 10.0);
    EXPECT_DOUBLE_EQ(z(1), 0.0);
    EXPECT_DOUBLE_EQ(z(2), 0.0);
    EXPECT_DOUBLE_EQ(z(3), -3.0);  // closing => negative range rate
}

TEST(RadarMeasurementModel, TangentialTargetZeroDoppler)
{
    const cuas::Vector6d x = makeState(10.0, 0.0, 0.0, 0.0, 2.0, 0.0);
    RadarMeasurementModel::MeasVector z;
    ASSERT_TRUE(RadarMeasurementModel::predict(x, z));
    EXPECT_DOUBLE_EQ(z(3), 0.0);
}

TEST(RadarMeasurementModel, JacobianMatchesFiniteDifferences)
{
    const cuas::Vector6d x = makeState(4.0, -3.0, 2.0, 1.5, -0.5, 2.5);
    RadarMeasurementModel::MeasJacobian H;
    ASSERT_TRUE(RadarMeasurementModel::jacobian(x, H));

    const cuas::float64_t h = 1.0e-6;
    for (Eigen::Index i = 0; i < 6; ++i) {
        cuas::Vector6d xp = x;
        cuas::Vector6d xm = x;
        xp(i) += h;
        xm(i) -= h;
        RadarMeasurementModel::MeasVector zp;
        RadarMeasurementModel::MeasVector zm;
        ASSERT_TRUE(RadarMeasurementModel::predict(xp, zp));
        ASSERT_TRUE(RadarMeasurementModel::predict(xm, zm));
        for (Eigen::Index r = 0; r < RadarMeasurementModel::kMeasDim; ++r) {
            const cuas::float64_t num = (zp(r) - zm(r)) / (2.0 * h);
            EXPECT_NEAR(H(r, i), num, 1.0e-5)
                << "row " << r << " col " << i;
        }
    }
}

TEST(RadarMeasurementModel, RejectsBelowMinRange)
{
    const cuas::Vector6d x = makeState(0.1, 0.0, 0.0, 1.0, 0.0, 0.0);
    RadarMeasurementModel::MeasVector z;
    RadarMeasurementModel::MeasJacobian H;
    EXPECT_FALSE(RadarMeasurementModel::predict(x, z));
    EXPECT_FALSE(RadarMeasurementModel::jacobian(x, H));
}

TEST(RadarMeasurementModel, RejectsNonFiniteState)
{
    const cuas::float64_t nan = std::numeric_limits<cuas::float64_t>::quiet_NaN();
    const cuas::Vector6d x = makeState(10.0, 0.0, 0.0, nan, 0.0, 0.0);
    RadarMeasurementModel::MeasVector z;
    RadarMeasurementModel::MeasJacobian H;
    EXPECT_FALSE(RadarMeasurementModel::predict(x, z));
    EXPECT_FALSE(RadarMeasurementModel::jacobian(x, H));
}

TEST(RadarMeasurementModel, NoiseIsDiagonalPositive)
{
    const RadarMeasurementModel::MeasMatrix R = RadarMeasurementModel::noise();
    for (Eigen::Index i = 0; i < RadarMeasurementModel::kMeasDim; ++i) {
        EXPECT_GT(R(i, i), 0.0);
        for (Eigen::Index j = 0; j < RadarMeasurementModel::kMeasDim; ++j) {
            if (i != j) {
                EXPECT_DOUBLE_EQ(R(i, j), 0.0);
            }
        }
    }
}

// --- Camera ---------------------------------------------------------------

cuas::ExtrinsicTransform identityExtrinsic()
{
    cuas::ExtrinsicTransform e;
    e.q_w = 1.0F;
    e.q_x = 0.0F;
    e.q_y = 0.0F;
    e.q_z = 0.0F;
    e.t_x_m = 0.0F;
    e.t_y_m = 0.0F;
    e.t_z_m = 0.0F;
    return e;
}

TEST(CameraBearingModel, BoresightTargetZeroBearing)
{
    CameraBearingModel m;
    ASSERT_TRUE(m.init(identityExtrinsic(), 1000.0, 1100.0, 640.0, 360.0));
    CameraBearingModel::MeasVector z;
    ASSERT_TRUE(m.predict(makeState(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), z));
    EXPECT_DOUBLE_EQ(z(0), 0.0);
    EXPECT_DOUBLE_EQ(z(1), 0.0);
}

TEST(CameraBearingModel, KnownRotationMapsBaseYToBoresight)
{
    // Default mount quaternion (90 deg about +x): base +y is camera +z.
    // A transposed rotation would put this target behind the camera.
    cuas::ExtrinsicTransform e = identityExtrinsic();
    e.q_w = 0.70710678F;
    e.q_x = 0.70710678F;
    CameraBearingModel m;
    ASSERT_TRUE(m.init(e, 1000.0, 1100.0, 640.0, 360.0));
    CameraBearingModel::MeasVector z;
    ASSERT_TRUE(m.predict(makeState(0.0, 5.0, 0.0, 0.0, 0.0, 0.0), z));
    EXPECT_NEAR(z(0), 0.0, 1.0e-6);
    EXPECT_NEAR(z(1), 0.0, 1.0e-6);
}

TEST(CameraBearingModel, OffAxisBearingSigns)
{
    CameraBearingModel m;
    ASSERT_TRUE(m.init(identityExtrinsic(), 1000.0, 1100.0, 640.0, 360.0));
    CameraBearingModel::MeasVector z;
    // Right of boresight: positive azimuth, exactly atan2(1, 5).
    ASSERT_TRUE(m.predict(makeState(1.0, 0.0, 5.0, 0.0, 0.0, 0.0), z));
    EXPECT_NEAR(z(0), std::atan2(1.0, 5.0), 1.0e-12);
    EXPECT_DOUBLE_EQ(z(1), 0.0);
    // Camera-frame -y is up: positive elevation.
    ASSERT_TRUE(m.predict(makeState(0.0, -1.0, 5.0, 0.0, 0.0, 0.0), z));
    EXPECT_DOUBLE_EQ(z(0), 0.0);
    EXPECT_NEAR(z(1), std::atan2(1.0, 5.0), 1.0e-12);
}

TEST(CameraBearingModel, PixelAndStateBearingsAgree)
{
    // The round-trip contract: a pixel generated from the state by the
    // same pinhole must yield the bearing predict() computes.
    const cuas::float64_t fx = 1000.0;
    const cuas::float64_t fy = 1100.0;
    const cuas::float64_t cx = 640.0;
    const cuas::float64_t cy = 360.0;
    cuas::ExtrinsicTransform e = identityExtrinsic();
    e.t_x_m = 0.1F;
    e.t_y_m = -0.2F;
    e.t_z_m = 0.05F;
    CameraBearingModel m;
    ASSERT_TRUE(m.init(e, fx, fy, cx, cy));

    const cuas::Vector6d x = makeState(2.0, 1.0, 8.0, 0.0, 0.0, 0.0);
    const cuas::float64_t xc = 2.0 + 0.1;
    const cuas::float64_t yc = 1.0 - 0.2;
    const cuas::float64_t zc = 8.0 + 0.05;
    const auto u = static_cast<cuas::float32_t>(fx * (xc / zc) + cx);
    const auto v = static_cast<cuas::float32_t>(fy * (yc / zc) + cy);

    CameraBearingModel::MeasVector z_px;
    CameraBearingModel::MeasVector z_state;
    ASSERT_TRUE(m.pixelToBearing(u, v, z_px));
    ASSERT_TRUE(m.predict(x, z_state));
    EXPECT_NEAR(z_px(0), z_state(0), 1.0e-5);
    EXPECT_NEAR(z_px(1), z_state(1), 1.0e-5);
}

TEST(CameraBearingModel, JacobianMatchesFiniteDifferences)
{
    cuas::ExtrinsicTransform e = identityExtrinsic();
    e.q_w = 0.70710678F;
    e.q_x = 0.70710678F;
    e.t_x_m = 0.1F;
    e.t_y_m = -0.2F;
    e.t_z_m = 0.05F;
    CameraBearingModel m;
    ASSERT_TRUE(m.init(e, 1000.0, 1100.0, 640.0, 360.0));

    const cuas::Vector6d x = makeState(1.5, 9.0, -0.8, 2.0, -1.0, 0.5);
    CameraBearingModel::MeasJacobian H;
    ASSERT_TRUE(m.jacobian(x, H));

    const cuas::float64_t h = 1.0e-6;
    for (Eigen::Index i = 0; i < 3; ++i) {
        cuas::Vector6d xp = x;
        cuas::Vector6d xm = x;
        xp(i) += h;
        xm(i) -= h;
        CameraBearingModel::MeasVector zp;
        CameraBearingModel::MeasVector zm;
        ASSERT_TRUE(m.predict(xp, zp));
        ASSERT_TRUE(m.predict(xm, zm));
        for (Eigen::Index r = 0; r < CameraBearingModel::kMeasDim; ++r) {
            const cuas::float64_t num = (zp(r) - zm(r)) / (2.0 * h);
            EXPECT_NEAR(H(r, i), num, 1.0e-5)
                << "row " << r << " col " << i;
        }
    }
    // Bearing carries no velocity information.
    for (Eigen::Index i = 3; i < 6; ++i) {
        for (Eigen::Index r = 0; r < CameraBearingModel::kMeasDim; ++r) {
            EXPECT_DOUBLE_EQ(H(r, i), 0.0);
        }
    }
}

TEST(CameraBearingModel, RejectsBehindOrTooCloseToCamera)
{
    CameraBearingModel m;
    ASSERT_TRUE(m.init(identityExtrinsic(), 1000.0, 1100.0, 640.0, 360.0));
    CameraBearingModel::MeasVector z;
    CameraBearingModel::MeasJacobian H;
    const cuas::Vector6d behind = makeState(0.0, 0.0, -5.0, 0.0, 0.0, 0.0);
    EXPECT_FALSE(m.predict(behind, z));
    EXPECT_FALSE(m.jacobian(behind, H));
    const cuas::Vector6d grazing = makeState(0.0, 0.0, 0.05, 0.0, 0.0, 0.0);
    EXPECT_FALSE(m.predict(grazing, z));
}

TEST(CameraBearingModel, InitRejectsDegenerateInputs)
{
    CameraBearingModel m;
    // Zero quaternion fails the extrinsicRotationMatrix contract.
    cuas::ExtrinsicTransform zero_q = identityExtrinsic();
    zero_q.q_w = 0.0F;
    EXPECT_FALSE(m.init(zero_q, 1000.0, 1100.0, 640.0, 360.0));
    // Non-positive / non-finite intrinsics.
    EXPECT_FALSE(m.init(identityExtrinsic(), 0.0, 1100.0, 640.0, 360.0));
    const cuas::float64_t nan = std::numeric_limits<cuas::float64_t>::quiet_NaN();
    EXPECT_FALSE(m.init(identityExtrinsic(), 1000.0, 1100.0, nan, 360.0));
    // A model with no successful init() stays unusable.
    CameraBearingModel::MeasVector z;
    EXPECT_FALSE(m.predict(makeState(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), z));
    EXPECT_FALSE(m.pixelToBearing(640.0F, 360.0F, z));
}

TEST(CameraBearingModel, NoiseScalesWithFocalLength)
{
    CameraBearingModel m;
    ASSERT_TRUE(m.init(identityExtrinsic(), 1000.0, 1100.0, 640.0, 360.0));
    const CameraBearingModel::MeasMatrix R = m.noise();
    const cuas::float64_t s_az = cuas::kCameraPixelSigmaPx / 1000.0;
    const cuas::float64_t s_el = cuas::kCameraPixelSigmaPx / 1100.0;
    EXPECT_DOUBLE_EQ(R(0, 0), s_az * s_az);
    EXPECT_DOUBLE_EQ(R(1, 1), s_el * s_el);
    EXPECT_DOUBLE_EQ(R(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(R(1, 0), 0.0);
}

}  // namespace
