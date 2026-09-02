// test_track_manager.cpp
// Unit tests for TrackManager: the A1.9 regressions — the Kalman gain must
// actually be applied to the state (no jump-to-measurement while P shrinks),
// and a single missed scan must not blank a confirmed track.

#include "cuas_fusion/tracking/track_manager.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace {

using cuas::FixedVector;
using cuas::FusedDetection;
using cuas::Track;
using cuas::TrackManager;
using cuas::TrackState;

constexpr int64_t kNs100ms = 100'000'000LL;

FusedDetection makeDetection(float x, float y, float z, int64_t ts_ns)
{
    FusedDetection det;
    det.position_x_m = x;
    det.position_y_m = y;
    det.position_z_m = z;
    det.velocity_mps = 5.0F;
    det.class_id     = 0;
    det.confidence   = 0.9F;
    det.timestamp_ns = ts_ns;
    return det;
}

// Feed n identical detections; returns the confirmed output of the last call.
FixedVector<Track, cuas::TRACK_MAX_TRACKS> feedHits(
    TrackManager& manager, float x, float y, float z, int32_t n, int64_t& ts)
{
    FixedVector<Track, cuas::TRACK_MAX_TRACKS> confirmed;
    for (int32_t k = 0; k < n; ++k) {
        ts += kNs100ms;
        const FusedDetection det = makeDetection(x, y, z, ts);
        EXPECT_TRUE(manager.update(&det, 1U, confirmed));
    }
    return confirmed;
}

TEST(TrackManager, ConfirmsAfterConfirmHitsAndConverges)
{
    TrackManager manager;
    ASSERT_TRUE(manager.init());
    int64_t ts = 0;

    const auto confirmed =
        feedHits(manager, 1.0F, 10.0F, 0.0F, cuas::TRACK_CONFIRM_HITS, ts);
    ASSERT_EQ(confirmed.size(), 1U);
    EXPECT_EQ(confirmed[0].state_, TrackState::CONFIRMED);
    // Identical measurements: the filtered state must sit on them exactly.
    EXPECT_NEAR(confirmed[0].position_x_m_, 1.0F, 1.0e-4F);
    EXPECT_NEAR(confirmed[0].position_y_m_, 10.0F, 1.0e-4F);
}

TEST(TrackManager, GainBlendsInsteadOfJumpingToMeasurement)
{
    TrackManager manager;
    ASSERT_TRUE(manager.init());
    int64_t ts = 0;

    // Converge on (1, 10, 0): P shrinks well below R after 5 hits.
    (void)feedHits(manager, 1.0F, 10.0F, 0.0F, 5, ts);

    // One offset measurement inside the confirmed gate. A consistent filter
    // moves the state a fraction of the innovation; the old code jumped the
    // state to 1.5 exactly while P claimed near-certainty.
    ts += kNs100ms;
    const FusedDetection det = makeDetection(1.5F, 10.0F, 0.0F, ts);
    FixedVector<Track, cuas::TRACK_MAX_TRACKS> confirmed;
    ASSERT_TRUE(manager.update(&det, 1U, confirmed));
    ASSERT_EQ(confirmed.size(), 1U);
    EXPECT_GT(confirmed[0].position_x_m_, 1.0F);
    EXPECT_LT(confirmed[0].position_x_m_, 1.3F);
}

TEST(TrackManager, ConfirmedTrackSurvivesSingleMiss)
{
    TrackManager manager;
    ASSERT_TRUE(manager.init());
    int64_t ts = 0;

    (void)feedHits(manager, 1.0F, 10.0F, 0.0F, cuas::TRACK_CONFIRM_HITS, ts);

    // One empty scan: the confirmed track must keep publishing (coast on its
    // last state), not blank out and re-earn confirmation over 3 scans.
    FixedVector<Track, cuas::TRACK_MAX_TRACKS> confirmed;
    ASSERT_TRUE(manager.update(nullptr, 0U, confirmed));
    ASSERT_EQ(confirmed.size(), 1U) << "single miss blanked a confirmed track";
    EXPECT_EQ(confirmed[0].state_, TrackState::CONFIRMED);

    // Next hit: still published.
    const auto after_hit = feedHits(manager, 1.0F, 10.0F, 0.0F, 1, ts);
    ASSERT_EQ(after_hit.size(), 1U);
    EXPECT_EQ(after_hit[0].state_, TrackState::CONFIRMED);
}

TEST(TrackManager, CoastedTrackRepublishesOnFirstHit)
{
    TrackManager manager;
    ASSERT_TRUE(manager.init());
    int64_t ts = 0;

    (void)feedHits(manager, 1.0F, 10.0F, 0.0F, cuas::TRACK_CONFIRM_HITS, ts);

    // Enough consecutive misses to demote to COASTED (but not delete).
    FixedVector<Track, cuas::TRACK_MAX_TRACKS> confirmed;
    for (int32_t k = 0; k < cuas::TRACK_COAST_MISSES; ++k) {
        ASSERT_TRUE(manager.update(nullptr, 0U, confirmed));
    }
    EXPECT_EQ(confirmed.size(), 0U) << "coasted track should not publish";

    // First hit after coasting: hit history was kept, so the track is
    // CONFIRMED and published immediately — no 3-scan re-confirmation gap.
    const auto after_hit = feedHits(manager, 1.0F, 10.0F, 0.0F, 1, ts);
    ASSERT_EQ(after_hit.size(), 1U);
    EXPECT_EQ(after_hit[0].state_, TrackState::CONFIRMED);
}

TEST(TrackManager, TrackDeletedAfterMaxMisses)
{
    TrackManager manager;
    ASSERT_TRUE(manager.init());
    int64_t ts = 0;

    (void)feedHits(manager, 1.0F, 10.0F, 0.0F, cuas::TRACK_CONFIRM_HITS, ts);

    FixedVector<Track, cuas::TRACK_MAX_TRACKS> confirmed;
    for (int32_t k = 0; k < cuas::TRACK_MAX_MISSES; ++k) {
        ASSERT_TRUE(manager.update(nullptr, 0U, confirmed));
    }
    EXPECT_EQ(confirmed.size(), 0U);

    // A new detection at the old position starts a fresh TENTATIVE track,
    // so nothing is confirmed on the first hit.
    const auto after_hit = feedHits(manager, 1.0F, 10.0F, 0.0F, 1, ts);
    EXPECT_EQ(after_hit.size(), 0U);
}

} // namespace
