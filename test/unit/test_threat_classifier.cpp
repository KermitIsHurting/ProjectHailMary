// test_threat_classifier.cpp
// Unit tests for ThreatClassifier rule engine.

#include "cuas_fusion/classification/threat_classifier.hpp"
#include "cuas_fusion/tracking/track.hpp"
#include "cuas_fusion/common/types.hpp"

#include <gtest/gtest.h>

// No defaulted TrackState: every test states the track state explicitly.
// (A defaulted CONFIRMED here silently added the +0.2 confirmed quality
// term to the "radar only" fixture — the original cause of the
// QualityScoreRadarOnly failure.)
static cuas::Track make_track(float confidence,
                               const std::string& class_label,
                               float velocity_mps,
                               float doppler_mps,
                               cuas::TrackState state)
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

// 1. Empty class label → UNKNOWN
TEST_F(ThreatClassifierTest, EmptyLabelReturnsUnknown)
{
    auto t = make_track(0.9f, "", 0.5f, 0.1f, cuas::TrackState::CONFIRMED);
    auto r = classifier_.classify(t, 1.0);
    EXPECT_EQ(r.threat_level, cuas::ThreatLevel::UNKNOWN);
}

// 2. Slow person → BENIGN
TEST_F(ThreatClassifierTest, SlowPersonReturnsBenign)
{
    auto t = make_track(0.9f, "0", 0.5f, 0.1f, cuas::TrackState::CONFIRMED);
    auto r = classifier_.classify(t, 1.0);
    EXPECT_EQ(r.threat_level, cuas::ThreatLevel::BENIGN);
}

// 3. Fast person → SUSPECT
TEST_F(ThreatClassifierTest, FastPersonReturnsSuspect)
{
    auto t = make_track(0.9f, "0", 3.0f, -1.0f, cuas::TrackState::CONFIRMED);
    auto r = classifier_.classify(t, 1.0);
    EXPECT_EQ(r.threat_level, cuas::ThreatLevel::SUSPECT);
}

// 4. Non-person high confidence → THREAT
TEST_F(ThreatClassifierTest, NonPersonHighConfReturnsThreat)
{
    auto t = make_track(0.9f, "14", 1.0f, 0.1f, cuas::TrackState::CONFIRMED);
    auto r = classifier_.classify(t, 1.0);
    EXPECT_EQ(r.threat_level, cuas::ThreatLevel::THREAT);
}

// 5. Non-person low confidence → UNKNOWN
TEST_F(ThreatClassifierTest, NonPersonLowConfReturnsUnknown)
{
    auto t = make_track(0.3f, "14", 1.0f, 0.1f, cuas::TrackState::CONFIRMED);
    auto r = classifier_.classify(t, 1.0);
    EXPECT_EQ(r.threat_level, cuas::ThreatLevel::UNKNOWN);
}

// 6. Quality score: radar only (no label, unconfirmed, no dwell) = base 0.3
TEST_F(ThreatClassifierTest, QualityScoreRadarOnly)
{
    auto t = make_track(0.9f, "", 0.5f, 0.1f, cuas::TrackState::TENTATIVE);
    auto r = classifier_.classify(t, 0.5);
    EXPECT_NEAR(r.quality_score, 0.3f, 0.01f);
}

// 7. Quality score: camera confirmed + CONFIRMED state + dwell > 3s
TEST_F(ThreatClassifierTest, QualityScoreFull)
{
    auto t = make_track(0.9f, "0", 0.5f, 0.1f, cuas::TrackState::CONFIRMED);
    // First call sets first_seen_s
    classifier_.classify(t, 0.0);
    // Second call at 4.0s → dwell > 3.0
    auto r = classifier_.classify(t, 4.0);
    EXPECT_NEAR(r.quality_score, 1.0f, 0.01f);
}
