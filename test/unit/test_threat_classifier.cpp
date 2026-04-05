// test_threat_classifier.cpp
// Unit tests for ThreatClassifier rule engine.

#include "cuas_fusion/classification/threat_classifier.hpp"
#include "cuas_fusion/tracking/track.hpp"
#include "cuas_fusion/common/types.hpp"

#include <gtest/gtest.h>

static cuas::Track make_track(float confidence,
                               const std::string& class_label,
                               float velocity_mps,
                               float doppler_mps,
                               cuas::TrackState state = cuas::TrackState::CONFIRMED)
{
    cuas::Track t;
    t.track_id_     = 1;
    t.confidence_   = confidence;
    t.class_label_  = class_label;
    t.velocity_mps_ = velocity_mps;
    t.doppler_mps_  = doppler_mps;
    t.state_        = state;
    return t;
}

class ThreatClassifierTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(classifier_.init());
    }
    cuas::ThreatClassifier classifier_;
};

// 1. Low confidence → UNKNOWN regardless of class.
TEST_F(ThreatClassifierTest, LowConfidenceReturnsUnknown)
{
    auto t = make_track(0.1f, "person", 0.5f, 0.1f);
    EXPECT_EQ(classifier_.classify(t), cuas::ThreatLevel::UNKNOWN);
}

// 2. Slow person not approaching → BENIGN.
TEST_F(ThreatClassifierTest, SlowPersonReturnsBenign)
{
    auto t = make_track(0.9f, "person", 0.5f, 0.1f);
    EXPECT_EQ(classifier_.classify(t), cuas::ThreatLevel::BENIGN);
}

// 3. Fast approaching target → THREAT.
TEST_F(ThreatClassifierTest, FastApproachingReturnsThreat)
{
    auto t = make_track(0.9f, "person", 6.0f, -1.0f);
    EXPECT_EQ(classifier_.classify(t), cuas::ThreatLevel::THREAT);
}

// 4. Bird class → SUSPECT (drone class triggers regardless of speed).
TEST_F(ThreatClassifierTest, DroneClassReturnsSuspect)
{
    // velocity < THREAT_VELOCITY_THREAT_MPS and not strongly approaching,
    // but bird is a drone class → SUSPECT via rule 4.
    auto t = make_track(0.9f, "bird", 1.0f, 0.1f);
    EXPECT_EQ(classifier_.classify(t), cuas::ThreatLevel::SUSPECT);
}

// 5. TENTATIVE state → UNKNOWN regardless of other params.
TEST_F(ThreatClassifierTest, TentativeTrackReturnsUnknown)
{
    auto t = make_track(0.9f, "bird", 8.0f, -2.0f, cuas::TrackState::TENTATIVE);
    EXPECT_EQ(classifier_.classify(t), cuas::ThreatLevel::UNKNOWN);
}
