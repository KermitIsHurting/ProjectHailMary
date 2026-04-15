// @file health_monitor.cpp
// @brief HealthMonitor implementation: EMA rate tracking and liveness scoring.
#include "cuas_fusion/health_monitor.hpp"

namespace cuas {

HealthMonitor::HealthMonitor()
: topics_()
{
    for (uint32_t i = 0U; i < kTopicCount; ++i) {
        const TopicHealth init{0.0F, 0.0F, 0.0F, kStatusDead};
        (void)topics_.push_back(init);
    }
}

void HealthMonitor::update(const uint32_t topic_id, const float32_t now_sec)
{
    if (topic_id >= kTopicCount) {
        return;
    }
    TopicHealth & t = topics_[topic_id];

    // WHY: on the first update last_recv_sec is 0 so dt becomes the uptime;
    // prime last_recv_sec without feeding a spurious 1/uptime sample into EMA.
    if (t.last_recv_sec <= 0.0F) {
        t.last_recv_sec = now_sec;
        t.status        = kStatusOk;
        return;
    }

    const float32_t dt = now_sec - t.last_recv_sec;

    if (dt > kDeadThresholdSec) {
        t.status = kStatusDead;
    } else if (dt > kStaleThresholdSec) {
        t.status = kStatusStale;
    } else {
        t.status = kStatusOk;
    }

    if (dt > 0.0F) {
        const float32_t instant_hz = 1.0F / dt;
        t.measured_hz = (kEmaAlpha * instant_hz)
                      + ((1.0F - kEmaAlpha) * t.measured_hz);
    }

    t.last_recv_sec = now_sec;
}

void HealthMonitor::refresh_status(const uint32_t topic_id, const float32_t now_sec)
{
    if (topic_id >= kTopicCount) {
        return;
    }
    TopicHealth & t = topics_[topic_id];
    if (t.last_recv_sec <= 0.0F) {
        t.status = kStatusDead;
        return;
    }
    const float32_t dt = now_sec - t.last_recv_sec;
    if (dt > kDeadThresholdSec) {
        t.status = kStatusDead;
    } else if (dt > kStaleThresholdSec) {
        t.status = kStatusStale;
    } else {
        t.status = kStatusOk;
    }
}

TopicHealth HealthMonitor::query(const uint32_t topic_id) const
{
    if (topic_id >= kTopicCount) {
        const TopicHealth empty{0.0F, 0.0F, 0.0F, kStatusDead};
        return empty;
    }
    return topics_[topic_id];
}

uint8_t HealthMonitor::overall_status() const
{
    bool any_stale = false;
    for (uint32_t i = 0U; i < kTopicCount; ++i) {
        const uint8_t s = topics_[i].status;
        if (s == kStatusDead) {
            return kOverallFailed;
        }
        if (s == kStatusStale) {
            any_stale = true;
        }
    }
    if (any_stale) {
        return kOverallDegraded;
    }
    return kOverallNominal;
}

void HealthMonitor::set_expected_hz(const uint32_t topic_id, const float32_t hz)
{
    if (topic_id >= kTopicCount) {
        return;
    }
    topics_[topic_id].expected_hz = hz;
}

} // namespace cuas
