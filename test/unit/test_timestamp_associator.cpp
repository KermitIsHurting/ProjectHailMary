// test_timestamp_associator.cpp
// Unit tests for TimestampAssociator: verifies nearest-neighbor lookup,
// the MAX_TIMESTAMP_DELTA_NS rejection threshold, circular eviction, and
// thread safety.
//
// The association window is a single requirement owned by
// constants.hpp:MAX_TIMESTAMP_DELTA_NS (150 ms per docs/latency_budget.md,
// widened from 50 ms in 8652f22 to absorb Orin Nano scheduling jitter).
// Tests derive their bounds from the constant so a requirement change
// cannot silently diverge from the test again.

#include "cuas_fusion/fusion/timestamp_associator.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <thread>
#include <chrono>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static cv::Mat make_dummy_frame()
{
    return cv::Mat::ones(4, 4, CV_8UC3);
}

static constexpr int64_t kWindowNs = cuas::MAX_TIMESTAMP_DELTA_NS;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(TimestampAssociator, EmptyBufferReturnsFalse)
{
    cuas::TimestampAssociator assoc;
    cv::Mat frame;
    int64_t ts = 0;
    EXPECT_FALSE(assoc.findBestMatch(0LL, frame, ts));
}

TEST(TimestampAssociator, SingleFrameRespectsWindowLimit)
{
    cuas::TimestampAssociator assoc;
    assoc.addCameraFrame(make_dummy_frame(), 0LL);

    cv::Mat frame;
    int64_t ts = 0;

    // Well inside the window
    EXPECT_TRUE(assoc.findBestMatch(kWindowNs / 2, frame, ts));
    EXPECT_EQ(ts, 0LL);

    // 1 ms past the window — must reject
    EXPECT_FALSE(assoc.findBestMatch(kWindowNs + 1'000'000LL, frame, ts));
}

TEST(TimestampAssociator, NearestFrameSelected)
{
    cuas::TimestampAssociator assoc;
    assoc.addCameraFrame(make_dummy_frame(),  0LL);
    assoc.addCameraFrame(make_dummy_frame(), 33'000'000LL);
    assoc.addCameraFrame(make_dummy_frame(), 66'000'000LL);

    cv::Mat frame;
    int64_t ts = 0;

    // Query at 50ms: nearest is 66ms (delta=16ms), not 33ms (delta=17ms)
    ASSERT_TRUE(assoc.findBestMatch(50'000'000LL, frame, ts));
    EXPECT_EQ(ts, 66'000'000LL);

    // Query at 20ms: nearest is 33ms (delta=13ms), not 0ms (delta=20ms)
    ASSERT_TRUE(assoc.findBestMatch(20'000'000LL, frame, ts));
    EXPECT_EQ(ts, 33'000'000LL);
}

TEST(TimestampAssociator, RejectsFramesBeyondWindow)
{
    cuas::TimestampAssociator assoc;
    assoc.addCameraFrame(make_dummy_frame(), 0LL);

    cv::Mat frame;
    int64_t ts = 0;

    // 1 ns over the window — must reject (strict-greater boundary)
    EXPECT_FALSE(assoc.findBestMatch(kWindowNs + 1LL, frame, ts));

    // Exactly at the window — must accept
    EXPECT_TRUE(assoc.findBestMatch(kWindowNs, frame, ts));
    EXPECT_EQ(ts, 0LL);

    // 1 ns under the window — must accept
    EXPECT_TRUE(assoc.findBestMatch(kWindowNs - 1LL, frame, ts));
    EXPECT_EQ(ts, 0LL);
}

TEST(TimestampAssociator, CircularBufferEvictsOldest)
{
    cuas::TimestampAssociator assoc;

    // Add 7 frames (buffer size is 6) — frame at t=0ms gets evicted
    for (int i = 0; i < 7; ++i) {
        assoc.addCameraFrame(make_dummy_frame(),
                             static_cast<int64_t>(i) * 1'000'000LL);
    }

    cv::Mat frame;
    int64_t ts = 0;

    // Query at t=0ms: t=0ms was evicted, nearest valid is t=1ms
    ASSERT_TRUE(assoc.findBestMatch(0LL, frame, ts));
    EXPECT_EQ(ts, 1'000'000LL);
    EXPECT_NE(ts, 0LL);
}

TEST(TimestampAssociator, ConcurrentAddAndFindNoDataRace)
{
    // Run with -DCMAKE_CXX_FLAGS="-fsanitize=thread" for TSAN validation
    cuas::TimestampAssociator assoc;

    std::thread writer([&] {
        for (int i = 0; i < 500; ++i) {
            assoc.addCameraFrame(cv::Mat::ones(4, 4, CV_8UC3),
                                 static_cast<int64_t>(i) * 1'000'000LL);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::thread reader([&] {
        for (int i = 0; i < 500; ++i) {
            cv::Mat frame;
            int64_t ts = 0;
            assoc.findBestMatch(
                static_cast<int64_t>(i) * 1'000'000LL + 500'000LL, frame, ts);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    writer.join();
    reader.join();
    SUCCEED();
}
