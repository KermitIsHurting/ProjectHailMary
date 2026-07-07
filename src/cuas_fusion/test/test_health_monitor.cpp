// @file test_health_monitor.cpp
// @brief Unit tests for HealthMonitor topic status logic.
#include "cuas_fusion/health_monitor.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <gtest/gtest.h>

// WHY: HealthMonitor::update() treats last_recv_ns <= 0 as uninitialized; shift
// the test timeline by +1.0s so that "t=0" becomes t=1.0 and the dt branch runs.

TEST(HealthMonitorTest, FreshUpdateOk)
{
    cuas::HealthMonitor hm;
    hm.update(cuas::kTopicRadar, 1'000'000'000LL);
    hm.update(cuas::kTopicRadar, 1'100'000'000LL);
    const cuas::TopicHealth h = hm.query(cuas::kTopicRadar);
    EXPECT_EQ(h.status, cuas::TopicStatus::kOk);
}

TEST(HealthMonitorTest, StaleAfterThreshold)
{
    cuas::HealthMonitor hm;
    hm.update(cuas::kTopicRadar, 1'000'000'000LL);
    hm.refresh_status(cuas::kTopicRadar, 1'600'000'000LL);
    const cuas::TopicHealth h = hm.query(cuas::kTopicRadar);
    EXPECT_EQ(h.status, cuas::TopicStatus::kStale);
}

TEST(HealthMonitorTest, DeadAfterThreshold)
{
    cuas::HealthMonitor hm;
    hm.update(cuas::kTopicRadar, 1'000'000'000LL);
    hm.refresh_status(cuas::kTopicRadar, 3'100'000'000LL);
    const cuas::TopicHealth h = hm.query(cuas::kTopicRadar);
    EXPECT_EQ(h.status, cuas::TopicStatus::kDead);
}

TEST(HealthMonitorTest, OverallNominal)
{
    cuas::HealthMonitor hm;
    for (cuas::uint32_t i = 0U; i < cuas::kTopicCount; ++i) {
        hm.update(i, 1'000'000'000LL);
        hm.update(i, 1'100'000'000LL);
    }
    EXPECT_EQ(hm.overall_status(), cuas::SystemStatus::kNominal);
}

TEST(HealthMonitorTest, OverallDegraded)
{
    cuas::HealthMonitor hm;
    for (cuas::uint32_t i = 0U; i < cuas::kTopicCount; ++i) {
        hm.update(i, 1'000'000'000LL);
        hm.update(i, 1'100'000'000LL);
    }
    hm.refresh_status(cuas::kTopicRadar, 1'700'000'000LL);
    EXPECT_EQ(hm.overall_status(), cuas::SystemStatus::kDegraded);
}

TEST(HealthMonitorTest, OverallFailed)
{
    cuas::HealthMonitor hm;
    for (cuas::uint32_t i = 0U; i < cuas::kTopicCount; ++i) {
        hm.update(i, 1'000'000'000LL);
        hm.update(i, 1'100'000'000LL);
    }
    hm.refresh_status(cuas::kTopicRadar, 3'200'000'000LL);
    EXPECT_EQ(hm.overall_status(), cuas::SystemStatus::kFailed);
}
