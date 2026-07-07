// test_fusion_engine.cpp
// Unit tests for radar-to-YOLO fusion: projection/association basics and the
// A1.8 regression — EMA state must be keyed by association identity, so two
// same-class targets never smear toward each other and stale states evict.

#include "cuas_fusion/fusion/fusion_engine.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace {

using cuas::BoundingBox;
using cuas::FixedVector;
using cuas::FusedDetection;
using cuas::FusionEngine;
using cuas::RadarDetection;

constexpr int64_t kNs100ms = 100'000'000LL;

RadarDetection makePoint(float x, float y, float z, float vel, int64_t ts_ns)
{
    RadarDetection rd;
    rd.x = x;
    rd.y = y;
    rd.z = z;
    rd.velocity = vel;
    rd.timestamp_ns = ts_ns;
    return rd;
}

// A box (in pixels) centred on the projection of radar point (x, y, z).
BoundingBox boxAround(float x, float y, float z, int32_t class_id)
{
    const float u = cuas::CAMERA_FX * (x / y) + cuas::CAMERA_CX;
    const float v = cuas::CAMERA_FY * (-z / y) + cuas::CAMERA_CY;
    BoundingBox bb;
    bb.x = u - 50.0F;
    bb.y = v - 50.0F;
    bb.w = 100.0F;
    bb.h = 100.0F;
    bb.confidence = 0.9F;
    bb.class_id   = class_id;
    return bb;
}

FusionEngine makeEngine()
{
    FusionEngine engine;
    cuas::ExtrinsicTransform ext{};  // zero offsets keep the test math exact
    EXPECT_TRUE(engine.init(ext));
    return engine;
}

TEST(FusionEngine, SinglePointFusesIntoMatchingBox)
{
    FusionEngine engine = makeEngine();

    FixedVector<RadarDetection, cuas::TRACK_MAX_TRACKS> radar;
    ASSERT_TRUE(radar.push_back(makePoint(-2.0F, 10.0F, 0.0F, 5.0F, kNs100ms)));

    FixedVector<BoundingBox, 128U> boxes;
    ASSERT_TRUE(boxes.push_back(boxAround(-2.0F, 10.0F, 0.0F, 3)));

    FixedVector<FusedDetection, cuas::TRACK_MAX_TRACKS> fused;
    ASSERT_TRUE(engine.projectAndAssociate(radar, boxes, fused));
    ASSERT_EQ(fused.size(), 1U);
    EXPECT_NEAR(fused[0].position_x_m, -2.0F, 1.0e-4F);
    EXPECT_NEAR(fused[0].position_y_m, 10.0F, 1.0e-4F);
    EXPECT_EQ(fused[0].class_label, "3");
}

TEST(FusionEngine, TwoSameClassTargetsDoNotConverge)
{
    FusionEngine engine = makeEngine();

    // Two class-0 targets 4 m apart. With the old class-keyed EMA they
    // shared one state and both drifted toward the x=0 midpoint.
    FixedVector<BoundingBox, 128U> boxes;
    ASSERT_TRUE(boxes.push_back(boxAround(-2.0F, 10.0F, 0.0F, 0)));
    ASSERT_TRUE(boxes.push_back(boxAround(2.0F, 10.0F, 0.0F, 0)));

    for (int32_t k = 0; k < 20; ++k) {
        const int64_t ts = (k + 1) * kNs100ms;
        FixedVector<RadarDetection, cuas::TRACK_MAX_TRACKS> radar;
        ASSERT_TRUE(radar.push_back(makePoint(-2.0F, 10.0F, 0.0F, 5.0F, ts)));
        ASSERT_TRUE(radar.push_back(makePoint(2.0F, 10.0F, 0.0F, 5.0F, ts)));

        FixedVector<FusedDetection, cuas::TRACK_MAX_TRACKS> fused;
        ASSERT_TRUE(engine.projectAndAssociate(radar, boxes, fused));
        ASSERT_EQ(fused.size(), 2U);

        // Measurements are constant, so each target must stay pinned to its
        // own position on every iteration — any drift means state sharing.
        const float x0 = std::min(fused[0].position_x_m, fused[1].position_x_m);
        const float x1 = std::max(fused[0].position_x_m, fused[1].position_x_m);
        EXPECT_NEAR(x0, -2.0F, 1.0e-3F) << "iteration " << k;
        EXPECT_NEAR(x1, 2.0F, 1.0e-3F) << "iteration " << k;
    }
}

TEST(FusionEngine, StaleEmaStateEvictsInsteadOfBlending)
{
    FusionEngine engine = makeEngine();

    FixedVector<BoundingBox, 128U> boxes;
    ASSERT_TRUE(boxes.push_back(boxAround(-2.0F, 10.0F, 0.0F, 0)));

    // Build up EMA state with velocity 5 m/s.
    int64_t ts = 0;
    for (int32_t k = 0; k < 5; ++k) {
        ts += kNs100ms;
        FixedVector<RadarDetection, cuas::TRACK_MAX_TRACKS> radar;
        ASSERT_TRUE(radar.push_back(makePoint(-2.0F, 10.0F, 0.0F, 5.0F, ts)));
        FixedVector<FusedDetection, cuas::TRACK_MAX_TRACKS> fused;
        ASSERT_TRUE(engine.projectAndAssociate(radar, boxes, fused));
        ASSERT_EQ(fused.size(), 1U);
    }

    // 2 s gap, then the same position with velocity 0. If the ghost state
    // survived, the output would be 0.65 * 5 = 3.25 m/s, not 0.
    ts += 2'000'000'000LL;
    FixedVector<RadarDetection, cuas::TRACK_MAX_TRACKS> radar;
    ASSERT_TRUE(radar.push_back(makePoint(-2.0F, 10.0F, 0.0F, 0.0F, ts)));
    FixedVector<FusedDetection, cuas::TRACK_MAX_TRACKS> fused;
    ASSERT_TRUE(engine.projectAndAssociate(radar, boxes, fused));
    ASSERT_EQ(fused.size(), 1U);
    EXPECT_NEAR(fused[0].velocity_mps, 0.0F, 1.0e-4F);
}

TEST(FusionEngine, DifferentClassesKeepSeparateStates)
{
    FusionEngine engine = makeEngine();

    // Two targets of different classes closer than the association gate:
    // class identity must still separate them.
    FixedVector<BoundingBox, 128U> boxes;
    ASSERT_TRUE(boxes.push_back(boxAround(-0.5F, 10.0F, 0.0F, 0)));
    ASSERT_TRUE(boxes.push_back(boxAround(0.5F, 10.0F, 0.0F, 1)));

    for (int32_t k = 0; k < 10; ++k) {
        const int64_t ts = (k + 1) * kNs100ms;
        FixedVector<RadarDetection, cuas::TRACK_MAX_TRACKS> radar;
        ASSERT_TRUE(radar.push_back(makePoint(-0.5F, 10.0F, 0.0F, 5.0F, ts)));
        ASSERT_TRUE(radar.push_back(makePoint(0.5F, 10.0F, 0.0F, 5.0F, ts)));

        FixedVector<FusedDetection, cuas::TRACK_MAX_TRACKS> fused;
        ASSERT_TRUE(engine.projectAndAssociate(radar, boxes, fused));
        ASSERT_EQ(fused.size(), 2U);
        const float x0 = std::min(fused[0].position_x_m, fused[1].position_x_m);
        const float x1 = std::max(fused[0].position_x_m, fused[1].position_x_m);
        EXPECT_NEAR(x0, -0.5F, 1.0e-3F) << "iteration " << k;
        EXPECT_NEAR(x1, 0.5F, 1.0e-3F) << "iteration " << k;
    }
}

} // namespace
