// test_imm_tracker.cpp
// Unit tests for IMMTracker association/velocity gates, confirmation
// counting and radial speed (audit B2: RC-1, RC-2, RC-3, RC-37), plus the
// PointCloud2 layout guards the tracker node applies (RC-8).

#include "cuas_fusion/tracking/imm_tracker.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/ros_pointcloud_adapter.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <random>

namespace {

constexpr double kDt    = 0.05;   // 20 Hz radar
constexpr double kEpoch = 1000.0; // arbitrary CLOCK_MONOTONIC seconds

struct StraightRun {
    double           speed_est;
    bool             single_track;   // every return gated to the one track
    cuas::TrackState state;
    uint32_t         rejects;
    double           radial;
};

// One target on a straight line at |v| = speed along -x (closing), returns
// with R-consistent noise, the node's 20 Hz predict interleaved.
StraightRun run_straight(double speed, unsigned seed, int frames)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> n(0.0, cuas::kRadarDetectionSigmaM);
    double t  = kEpoch;
    // Start far enough out that the target is still closing after `frames`
    // (a 15 m/s target covers 30 m in the 40-frame run).
    double px = 5.0 + (2.5 * speed);
    const double py = 2.0;
    const double pz = 1.5;
    cuas::IMMTracker trk(1U, px + n(rng), py + n(rng), pz + n(rng), t);
    bool single = true;
    for (int k = 0; k < frames; ++k) {
        t  += kDt;
        px -= speed * kDt;
        trk.predict(kDt);
        const double mx = px + n(rng);
        const double my = py + n(rng);
        const double mz = pz + n(rng);
        if (!trk.gates(mx, my, mz, t)) {
            single = false;
        }
        trk.update(mx, my, mz, t);
    }
    return {trk.speed(), single, trk.getState(), trk.getVelocityRejectCount(),
            trk.radialSpeed()};
}

sensor_msgs::msg::PointCloud2 make_cloud(bool with_z, uint32_t width)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1U;
    msg.width  = width;
    sensor_msgs::PointCloud2Modifier mod(msg);
    if (with_z) {
        mod.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    } else {
        mod.setPointCloud2Fields(2,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32);
    }
    mod.resize(width);
    return msg;
}

} // namespace

TEST(ImmTracker, StraightTargetsStayOneTrackAndConvergeSpeed)
{
    for (const double v : {1.0, 5.0, 10.0, 15.0}) {
        for (const unsigned seed : {1U, 2U, 3U}) {
            const StraightRun r = run_straight(v, seed, 40);
            EXPECT_TRUE(r.single_track) << v << " m/s, seed " << seed;
            EXPECT_NEAR(r.speed_est, v, std::max(0.10 * v, 0.3))
                << v << " m/s, seed " << seed << ", rejects " << r.rejects;
            EXPECT_EQ(r.state, cuas::TrackState::CONFIRMED) << v << " m/s";
            EXPECT_LT(r.radial, 0.0) << "closing target must read negative";
        }
    }
}

TEST(ImmTracker, VelocityGateAdmitsFastTargetFromZeroInit)
{
    // RC-2: the fixed 3 m/s^2 * dt gate pinned a 10 m/s target near 0.17 m/s.
    const StraightRun r = run_straight(10.0, 7U, 5);
    EXPECT_GT(r.speed_est, 5.0);
}

TEST(ImmTracker, RadialSpeedSignFollowsTiConvention)
{
    // Receding along +x from (10, 0, 0): positive.
    double t = kEpoch;
    double px = 10.0;
    cuas::IMMTracker trk(1U, px, 0.0, 0.0, t);
    for (int k = 0; k < 10; ++k) {
        t += kDt;
        px += 3.0 * kDt;
        trk.predict(kDt);
        trk.update(px, 0.0, 0.0, t);
    }
    EXPECT_GT(trk.radialSpeed(), 1.0);
    EXPECT_NEAR(trk.radialSpeed(), trk.getVelocity().x(), 0.2);
}

TEST(ImmTracker, RadialSpeedZeroAtSensorOrigin)
{
    cuas::IMMTracker trk(1U, 0.0, 0.0, 0.0, kEpoch);
    EXPECT_DOUBLE_EQ(trk.radialSpeed(), 0.0);
}

TEST(ImmTracker, OneHitPerStampAndGapRestartsConfirmation)
{
    // RC-37: five returns of ONE cloud (same stamp) must not confirm.
    cuas::IMMTracker trk(1U, 5.0, 0.0, 1.0, kEpoch);
    for (int k = 0; k < 5; ++k) {
        trk.update(5.0, 0.0, 1.0, kEpoch);
    }
    EXPECT_EQ(trk.getState(), cuas::TrackState::TENTATIVE);

    // Two more distinct frames (birth + 2 = 3 hits), then a gap longer than
    // kTrackHitGapResetS: the count restarts.
    double t = kEpoch;
    for (int k = 0; k < 2; ++k) {
        t += kDt;
        trk.update(5.0, 0.0, 1.0, t);
    }
    EXPECT_EQ(trk.getState(), cuas::TrackState::TENTATIVE);
    t += cuas::kTrackHitGapResetS + 0.5;
    for (int k = 0; k < 3; ++k) {
        t += kDt;
        trk.update(5.0, 0.0, 1.0, t);
    }
    EXPECT_EQ(trk.getState(), cuas::TrackState::TENTATIVE)
        << "3 hits after a gap must not reach the 5-hit confirmation";
    for (int k = 0; k < 2; ++k) {
        t += kDt;
        trk.update(5.0, 0.0, 1.0, t);
    }
    EXPECT_EQ(trk.getState(), cuas::TrackState::CONFIRMED);
}

TEST(ImmTracker, AssociationGateWidensForFreshTrackAndTightensWhenConverged)
{
    cuas::IMMTracker trk(1U, 5.0, 0.0, 1.0, kEpoch);
    const double fresh_gate = trk.associationGateM(kEpoch + kDt);
    EXPECT_GT(fresh_gate, cuas::kAssocBaseGateM);
    EXPECT_LE(fresh_gate, cuas::kAssocMaxGateM);

    double t = kEpoch;
    for (int k = 0; k < 40; ++k) {
        t += kDt;
        trk.predict(kDt);
        trk.update(5.0, 0.0, 1.0, t);
    }
    const double converged_gate = trk.associationGateM(t);
    EXPECT_LT(converged_gate, fresh_gate);
    EXPECT_GE(converged_gate, cuas::kAssocBaseGateM);

    // A return 0.5 m away from a converged stationary track gates; one at
    // the capped maximum plus margin never does.
    EXPECT_TRUE(trk.gates(5.5, 0.0, 1.0, t));
    EXPECT_FALSE(trk.gates(5.0 + cuas::kAssocMaxGateM + 0.1, 0.0, 1.0, t));
}

TEST(ImmTracker, PredictedPositionExtrapolatesToStamp)
{
    // Converge on a 4 m/s target, then ask for the position one frame ahead.
    double t = kEpoch;
    double px = 10.0;
    cuas::IMMTracker trk(1U, px, 0.0, 1.0, t);
    for (int k = 0; k < 40; ++k) {
        t += kDt;
        px -= 4.0 * kDt;
        trk.predict(kDt);
        trk.update(px, 0.0, 1.0, t);
    }
    const Eigen::Vector3d ahead = trk.predictedPositionAt(t + kDt);
    EXPECT_NEAR(ahead.x(), px - 4.0 * kDt, 0.15);
}

TEST(PointCloudGuards, RejectMissingZAndDetectEmpty)
{
    const auto ok    = make_cloud(true, 2U);
    const auto no_z  = make_cloud(false, 2U);
    const auto empty = make_cloud(true, 0U);
    EXPECT_TRUE(cuas::cloudHasFloat32Xyz(ok));
    EXPECT_FALSE(cuas::cloudHasFloat32Xyz(no_z));
    EXPECT_FALSE(cuas::cloudIsEmpty(ok));
    EXPECT_TRUE(cuas::cloudIsEmpty(empty));
    EXPECT_TRUE(cuas::cloudHasFloat32Xyz(empty)) << "layout check is independent of size";
}
