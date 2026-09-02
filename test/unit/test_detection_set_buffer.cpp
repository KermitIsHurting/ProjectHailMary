// test_detection_set_buffer.cpp
// Unit tests for the stamped detection-set ring behind P2.2 fusion
// alignment: nearest-stamp selection (not latest), inclusive window edge,
// empty sets as valid observations, and ring eviction. The window-edge
// cases exist because the original TimestampAssociator shipped with a
// gate-boundary bug — the boundary is tested explicitly here.

#include "cuas_fusion/fusion/detection_set_buffer.hpp"

#include <gtest/gtest.h>

namespace {

using cuas::BoundingBox;
using cuas::DetectionSetBuffer;

constexpr int64_t kMs = 1'000'000LL;

DetectionSetBuffer::BoxSet oneBox(float x)
{
    DetectionSetBuffer::BoxSet set;
    BoundingBox bb;
    bb.x = x;
    bb.w = 10.0F;
    bb.h = 10.0F;
    EXPECT_TRUE(set.push_back(bb));
    return set;
}

TEST(DetectionSetBuffer, EmptyBufferReturnsFalse)
{
    const DetectionSetBuffer buf;
    DetectionSetBuffer::BoxSet out;
    int64_t ts = -1;
    EXPECT_FALSE(buf.selectNearest(100 * kMs, 150 * kMs, out, ts));
    EXPECT_EQ(ts, -1);  // outputs untouched on failure
}

TEST(DetectionSetBuffer, SelectsNearestNotLatest)
{
    DetectionSetBuffer buf;
    buf.addSet(oneBox(1.0F), 100 * kMs);
    buf.addSet(oneBox(2.0F), 150 * kMs);
    buf.addSet(oneBox(3.0F), 220 * kMs);

    DetectionSetBuffer::BoxSet out;
    int64_t ts = 0;
    ASSERT_TRUE(buf.selectNearest(160 * kMs, 150 * kMs, out, ts));
    EXPECT_EQ(ts, 150 * kMs);  // 10 ms away beats 60 ms — nearest wins
    ASSERT_EQ(out.size(), 1U);
    EXPECT_FLOAT_EQ(out[0].x, 2.0F);
}

TEST(DetectionSetBuffer, WindowEdgeIsInclusiveAndBeyondRejects)
{
    DetectionSetBuffer buf;
    buf.addSet(oneBox(1.0F), 100 * kMs);

    DetectionSetBuffer::BoxSet out;
    int64_t ts = 0;
    // Exactly at the window edge: a match (inclusive, like the associator).
    EXPECT_TRUE(buf.selectNearest(250 * kMs, 150 * kMs, out, ts));
    // One nanosecond beyond: rejected.
    EXPECT_FALSE(buf.selectNearest(250 * kMs + 1, 150 * kMs, out, ts));
}

TEST(DetectionSetBuffer, EmptySetIsAValidObservation)
{
    DetectionSetBuffer buf;
    buf.addSet(oneBox(1.0F), 100 * kMs);
    buf.addSet(DetectionSetBuffer::BoxSet{}, 133 * kMs);  // frame saw nothing

    DetectionSetBuffer::BoxSet out;
    int64_t ts = 0;
    ASSERT_TRUE(buf.selectNearest(135 * kMs, 150 * kMs, out, ts));
    EXPECT_EQ(ts, 133 * kMs);
    EXPECT_TRUE(out.empty());  // the "no targets" frame wins, boxes cleared
}

TEST(DetectionSetBuffer, RingEvictsOldestEntries)
{
    DetectionSetBuffer buf;
    // Fill capacity + 2 so the two oldest sets are overwritten.
    for (int64_t i = 0; i < static_cast<int64_t>(cuas::TIMESTAMP_BUFFER_SIZE) + 2; ++i) {
        buf.addSet(oneBox(static_cast<float>(i)), (100 + i * 33) * kMs);
    }

    DetectionSetBuffer::BoxSet out;
    int64_t ts = 0;
    // Target at the first (evicted) stamp: nearest survivor is entry #2.
    ASSERT_TRUE(buf.selectNearest(100 * kMs, 150 * kMs, out, ts));
    EXPECT_EQ(ts, (100 + 2 * 33) * kMs);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_FLOAT_EQ(out[0].x, 2.0F);
}

} // namespace
