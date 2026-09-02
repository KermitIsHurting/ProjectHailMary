// test_threat_classifier.cpp
// Unit tests for ThreatClassifier rule engine.

#include "cuas_fusion/classification/label_join.hpp"
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
    t.class_id_     = cuas::parseClassId(class_label);
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

// 8. retainOnly drops states for ids the tracker no longer publishes (RC-4):
//    a reused/absent id restarts its dwell, a retained id keeps it.
TEST_F(ThreatClassifierTest, RetainOnlyDropsStatesOfAbsentTracks)
{
    auto t1 = make_track(0.9f, "0", 0.5f, 0.1f, cuas::TrackState::CONFIRMED);
    auto t2 = t1;
    t2.track_id_ = 2;
    classifier_.classify(t1, 0.0);
    classifier_.classify(t2, 0.0);

    cuas::FixedVector<uint32_t, cuas::TRACK_MAX_TRACKS> ids;
    ASSERT_TRUE(ids.push_back(1U));
    classifier_.retainOnly(ids);

    const auto r1 = classifier_.classify(t1, 4.0);
    const auto r2 = classifier_.classify(t2, 4.0);
    EXPECT_NEAR(r1.dwell_time_s, 4.0f, 0.01f) << "retained id keeps its dwell";
    EXPECT_NEAR(r2.dwell_time_s, 0.0f, 0.01f) << "dropped id starts over";
}

// 9. 40 distinct ids through a 32-slot map: with retainOnly after each
//    frame, the newest id always gets a state (the old failure was silent
//    starvation once 32 dead ids filled the map).
TEST_F(ThreatClassifierTest, RetainOnlyPreventsSlotStarvation)
{
    for (uint32_t id = 1U; id <= 40U; ++id) {
        auto t = make_track(0.9f, "0", 0.5f, 0.1f, cuas::TrackState::CONFIRMED);
        t.track_id_ = id;
        classifier_.classify(t, static_cast<double>(id));
        const auto r = classifier_.classify(t, static_cast<double>(id) + 2.0);
        EXPECT_EQ(r.escalation_state, cuas::EscalationState::TRACKED) << "id " << id;
        cuas::FixedVector<uint32_t, cuas::TRACK_MAX_TRACKS> ids;
        ASSERT_TRUE(ids.push_back(id));
        classifier_.retainOnly(ids);
    }
}

// 10. R6 F1: a 10 m/s target whose fused set is 100 ms newer than the track
//     sits 1 m from its own label unless the track is extrapolated first.
TEST(LabelJoin, ExtrapolatesTrackToFusedInstant)
{
    cuas::LabelJoinTrack t;
    t.x_m = 0.0F; t.y_m = 10.0F; t.z_m = 1.0F;
    t.vx = 10.0F; t.vy = 0.0F;   t.vz = 0.0F;
    t.stamp_ns = 1'000'000'000LL;
    cuas_msgs::msg::FusedDetection fd;
    fd.position_x_m = 1.0F;   // where the target IS 100 ms later
    fd.position_y_m = 10.0F;
    fd.position_z_m = 1.0F;
    fd.azimuth_deg  = cuas::bearingDegBoresightZero(1.0F, 10.0F);
    fd.class_label  = "0";
    const std::vector<cuas_msgs::msg::FusedDetection> dets{fd};
    EXPECT_EQ(cuas::joinFusedLabel(t, dets, 1'100'000'000LL, 1.0F, 15.0F), 0);
    // Same set, but the track is stationary: 1 m away, at the gate edge → joins;
    // 1.2 m away with a stationary track → no join.
    t.vx = 0.0F;
    EXPECT_EQ(cuas::joinFusedLabel(t, dets, 1'100'000'000LL, 0.9F, 15.0F), -1);
    // R6b-1: the track position is valid at the ARRAY stamp (already
    // predicted forward); a fused set 50 ms older than that must pull the
    // track BACK 0.5 m. Gate 0.3 m discriminates corrected vs uncorrected.
    cuas::LabelJoinTrack pub;
    pub.x_m = 1.0F; pub.y_m = 10.0F; pub.z_m = 1.0F;
    pub.vx = 10.0F; pub.stamp_ns = 1'100'000'000LL;   // header stamp
    cuas_msgs::msg::FusedDetection older = fd;
    older.position_x_m = 0.5F;
    older.azimuth_deg  = cuas::bearingDegBoresightZero(0.5F, 10.0F);
    cuas_msgs::msg::FusedDetection uncorrected = fd;   // sits at x = 1.0
    const std::vector<cuas_msgs::msg::FusedDetection> dets3{older};
    const std::vector<cuas_msgs::msg::FusedDetection> dets4{uncorrected};
    EXPECT_EQ(cuas::joinFusedLabel(pub, dets3, 1'050'000'000LL, 0.3F, 15.0F), 0);
    EXPECT_EQ(cuas::joinFusedLabel(pub, dets4, 1'050'000'000LL, 0.3F, 15.0F), -1);

    // Bearing mismatch rejects even a near position.
    cuas_msgs::msg::FusedDetection wrong_az = fd;
    wrong_az.azimuth_deg = 60.0F;
    const std::vector<cuas_msgs::msg::FusedDetection> dets2{wrong_az};
    t.vx = 10.0F;
    EXPECT_EQ(cuas::joinFusedLabel(t, dets2, 1'100'000'000LL, 1.0F, 15.0F), -1);
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
