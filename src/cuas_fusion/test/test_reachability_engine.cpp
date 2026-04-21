// @file test_reachability_engine.cpp
// @brief Unit tests for ReachabilityEngine intercept math.
#include "cuas_fusion/reachability_engine.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <gtest/gtest.h>

namespace {

cuas::ReachabilityTrackState make_state(const cuas::float32_t x,
                                        const cuas::float32_t y,
                                        const cuas::float32_t vx,
                                        const cuas::float32_t vy,
                                        const cuas::float32_t imm_cv_weight)
{
    cuas::ReachabilityTrackState s;
    s.x_m           = x;
    s.y_m           = y;
    s.vx_mps        = vx;
    s.vy_mps        = vy;
    s.imm_cv_weight = imm_cv_weight;
    for (cuas::uint32_t i = 0U; i < 4U; ++i) {
        for (cuas::uint32_t j = 0U; j < 4U; ++j) {
            s.P[i][j] = 0.0F;
        }
    }
    s.P[0][0] = 1.0F;
    s.P[1][1] = 1.0F;
    s.P[2][2] = 1.0F;
    s.P[3][3] = 1.0F;
    return s;
}

} // namespace

TEST(ReachabilityEngineTest, ApproachingTrack)
{
    const cuas::ReachabilityEngine engine;
    const cuas::ReachabilityTrackState s = make_state(5.0F, 0.0F, -1.0F, 0.0F, 0.8F);
    const cuas::InterceptResult r = engine.compute(s);
    EXPECT_TRUE(r.intercept_possible);
    EXPECT_NEAR(r.time_to_intercept_s, 5.0F, 0.1F);
}

TEST(ReachabilityEngineTest, DepartingTrack)
{
    const cuas::ReachabilityEngine engine;
    const cuas::ReachabilityTrackState s = make_state(5.0F, 0.0F, 1.0F, 0.0F, 0.8F);
    const cuas::InterceptResult r = engine.compute(s);
    EXPECT_FALSE(r.intercept_possible);
}

TEST(ReachabilityEngineTest, StationaryTrack)
{
    const cuas::ReachabilityEngine engine;
    const cuas::ReachabilityTrackState s = make_state(5.0F, 0.0F, 0.0F, 0.0F, 0.8F);
    const cuas::InterceptResult r = engine.compute(s);
    EXPECT_FALSE(r.intercept_possible);
}

TEST(ReachabilityEngineTest, ConfidenceClamped)
{
    const cuas::ReachabilityEngine engine;
    const cuas::ReachabilityTrackState s = make_state(5.0F, 0.0F, -1.0F, 0.0F, 1.5F);
    const cuas::InterceptResult r = engine.compute(s);
    EXPECT_FLOAT_EQ(r.intercept_confidence, 1.0F);
}

TEST(ReachabilityEngineTest, ConfidenceZero)
{
    const cuas::ReachabilityEngine engine;
    const cuas::ReachabilityTrackState s = make_state(5.0F, 0.0F, -1.0F, 0.0F, 0.0F);
    const cuas::InterceptResult r = engine.compute(s);
    EXPECT_FLOAT_EQ(r.intercept_confidence, 0.0F);
}

TEST(ReachabilityEngineTest, EllipseAxesPositive)
{
    const cuas::ReachabilityEngine engine;
    cuas::ReachabilityTrackState s = make_state(5.0F, 0.0F, -1.0F, 0.0F, 0.8F);
    s.P[0][0] = 4.0F;
    s.P[0][1] = 0.5F;
    s.P[1][0] = 0.5F;
    s.P[1][1] = 2.0F;
    const cuas::InterceptResult r = engine.compute(s);
    EXPECT_GE(r.ellipse_major_m, r.ellipse_minor_m);
    EXPECT_GE(r.ellipse_minor_m, 0.0F);
}
