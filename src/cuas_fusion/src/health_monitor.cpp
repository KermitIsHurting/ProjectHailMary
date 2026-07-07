// @file health_monitor.cpp
// @brief HealthMonitor implementation: EMA rate tracking and liveness scoring.
#include "cuas_fusion/health_monitor.hpp"

namespace cuas {

HealthMonitor::HealthMonitor()
: topics_()
{
    for (uint32_t i = 0U; i < kTopicCount; ++i) {
        const TopicHealth init{0LL, 0.0F, 0.0F, TopicStatus::kDead};
        (void)topics_.push_back(init);
    }
}

void HealthMonitor::update(const uint32_t topic_id, const int64_t now_ns)
{
    if (topic_id >= kTopicCount) {
        return;
    }
    TopicHealth & t = topics_[topic_id];

    // WHY: on the first update last_recv_ns is 0 so dt becomes the uptime;
    // prime last_recv_ns without feeding a spurious 1/uptime sample into EMA.
    if (t.last_recv_ns <= 0LL) {
        t.last_recv_ns = now_ns;
        t.status       = TopicStatus::kOk;
        return;
    }

    const int64_t dt_ns = now_ns - t.last_recv_ns;

    if (dt_ns > kDeadThresholdNs) {
        t.status = TopicStatus::kDead;
    } else if (dt_ns > kStaleThresholdNs) {
        t.status = TopicStatus::kStale;
    } else {
        t.status = TopicStatus::kOk;
    }

    if (dt_ns > 0LL) {
        // Subtract-then-narrow: the difference is small, so the float
        // conversion is exact where it matters.
        const float32_t dt_s = static_cast<float32_t>(dt_ns) * 1.0e-9F;
        const float32_t instant_hz = 1.0F / dt_s;
        t.measured_hz = (kEmaAlpha * instant_hz)
                      + ((1.0F - kEmaAlpha) * t.measured_hz);
    }

    t.last_recv_ns = now_ns;
}

void HealthMonitor::refresh_status(const uint32_t topic_id, const int64_t now_ns)
{
    if (topic_id >= kTopicCount) {
        return;
    }
    TopicHealth & t = topics_[topic_id];
    if (t.last_recv_ns <= 0LL) {
        t.status = TopicStatus::kDead;
        return;
    }
    const int64_t dt_ns = now_ns - t.last_recv_ns;
    if (dt_ns > kDeadThresholdNs) {
        t.status = TopicStatus::kDead;
    } else if (dt_ns > kStaleThresholdNs) {
        t.status = TopicStatus::kStale;
    } else {
        t.status = TopicStatus::kOk;
    }
}

TopicHealth HealthMonitor::query(const uint32_t topic_id) const
{
    if (topic_id >= kTopicCount) {
        const TopicHealth empty{0LL, 0.0F, 0.0F, TopicStatus::kDead};
        return empty;
    }
    return topics_[topic_id];
}

SystemStatus HealthMonitor::overall_status() const
{
    bool any_stale = false;
    for (uint32_t i = 0U; i < kTopicCount; ++i) {
        const TopicStatus s = topics_[i].status;
        if (s == TopicStatus::kDead) {
            return SystemStatus::kFailed;
        }
        if (s == TopicStatus::kStale) {
            any_stale = true;
        }
    }
    if (any_stale) {
        return SystemStatus::kDegraded;
    }
    return SystemStatus::kNominal;
}

void HealthMonitor::set_expected_hz(const uint32_t topic_id, const float32_t hz)
{
    if (topic_id >= kTopicCount) {
        return;
    }
    topics_[topic_id].expected_hz = hz;
}

} // namespace cuas
