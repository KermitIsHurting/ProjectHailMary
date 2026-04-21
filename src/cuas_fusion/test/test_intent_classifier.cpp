// @file test_intent_classifier.cpp
// @brief Unit tests for IntentClassifier behavioral logic.
#include "cuas_fusion/intent_classifier.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/intent_ids.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace {

cuas::IntentInput make_input(const cuas::float32_t x,
                             const cuas::float32_t y,
                             const cuas::float32_t vx,
                             const cuas::float32_t vy)
{
    cuas::IntentInput in;
    in.track_id  = 1U;
    in.x_m       = x;
    in.y_m       = y;
    in.vx_mps    = vx;
    in.vy_mps    = vy;
    in.speed_mps = std::sqrt((vx * vx) + (vy * vy));
    return in;
}

} // namespace

TEST(IntentClassifierTest, Approaching)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput in = make_input(3.0F, 0.0F, -1.0F, 0.0F);
    const cuas::IntentResult r = c.classify(in);
    EXPECT_EQ(r.intent, cuas::intent_class::kApproaching);
    EXPECT_GT(r.confidence, 0.0F);
}

TEST(IntentClassifierTest, Departing)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput in = make_input(3.0F, 0.0F, 1.0F, 0.0F);
    const cuas::IntentResult r = c.classify(in);
    EXPECT_EQ(r.intent, cuas::intent_class::kDeparting);
    EXPECT_GT(r.confidence, 0.0F);
}

TEST(IntentClassifierTest, Loitering)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput in = make_input(3.0F, 0.0F, 0.1F, 0.1F);
    const cuas::IntentResult r = c.classify(in);
    EXPECT_EQ(r.intent, cuas::intent_class::kLoitering);
}

TEST(IntentClassifierTest, Orbiting)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput in = make_input(3.0F, 0.0F, 0.0F, 1.0F);
    const cuas::IntentResult r = c.classify(in);
    EXPECT_EQ(r.intent, cuas::intent_class::kOrbiting);
}

// WHY: at distance 3 with any speed>=0.3 the ORBITING branch fires first; to
// reach TRANSITING the distance must be outside [kOrbitRadiusMin, kOrbitRadiusMax].
TEST(IntentClassifierTest, Transiting)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput in = make_input(12.0F, 0.0F, 0.0F, 2.0F);
    const cuas::IntentResult r = c.classify(in);
    EXPECT_EQ(r.intent, cuas::intent_class::kTransiting);
}

TEST(IntentClassifierTest, UnknownAtOrigin)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput in = make_input(0.0F, 0.0F, 0.0F, 0.0F);
    const cuas::IntentResult r = c.classify(in);
    EXPECT_EQ(r.intent, cuas::intent_class::kUnknown);
}

TEST(IntentClassifierTest, ConfidenceRange)
{
    const cuas::IntentClassifier c;
    const cuas::IntentInput cases[6] = {
        make_input( 3.0F,  0.0F, -1.0F, 0.0F),
        make_input( 3.0F,  0.0F,  1.0F, 0.0F),
        make_input( 3.0F,  0.0F,  0.1F, 0.1F),
        make_input( 3.0F,  0.0F,  0.0F, 1.0F),
        make_input(12.0F,  0.0F,  0.0F, 2.0F),
        make_input( 0.0F,  0.0F,  0.0F, 0.0F),
    };
    for (cuas::uint32_t i = 0U; i < 6U; ++i) {
        const cuas::IntentResult r = c.classify(cases[i]);
        EXPECT_GE(r.confidence, 0.0F);
        EXPECT_LE(r.confidence, 1.0F);
    }
}
