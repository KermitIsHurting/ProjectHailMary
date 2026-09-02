// @file health_monitor.hpp
// @brief Pure-math tracker of per-topic receive rates and liveness states.
#pragma once

#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

static constexpr uint32_t kTopicRadar      = 0U;
static constexpr uint32_t kTopicCamera     = 1U;
static constexpr uint32_t kTopicTracker    = 2U;
static constexpr uint32_t kTopicClassifier = 3U;
static constexpr uint32_t kTopicPredictor  = 4U;
static constexpr uint32_t kTopicCount      = 5U;

enum class TopicStatus : uint8_t {
    kOk    = 0U,
    kStale = 1U,
    kDead  = 2U
};

enum class SystemStatus : uint8_t {
    kNominal  = 0U,
    kDegraded = 1U,
    kFailed   = 2U
};

struct TopicHealth {
    // int64 nanoseconds, not float32 seconds: a float32 epoch-seconds value
    // loses sub-second resolution after ~2^24 s (~194 days of uptime) and
    // the staleness watchdog silently rots (A6.8). Subtract in integer ns,
    // then narrow the small difference to float for the Hz math.
    int64_t     last_recv_ns;
    float32_t   expected_hz;
    float32_t   measured_hz;
    TopicStatus status;
};

class HealthMonitor {
public:
    static constexpr int64_t   kStaleThresholdNs = 500'000'000LL;
    static constexpr int64_t   kDeadThresholdNs  = 2'000'000'000LL;
    static constexpr float32_t kEmaAlpha         = 0.1F;
    static constexpr int64_t   kMinRateSampleNs  = 5'000'000LL;

    HealthMonitor();

    void update(const uint32_t topic_id, const int64_t now_ns);
    // Expected silence: the producer has nothing to publish (e.g. the
    // predictor with zero confirmed tracks). Keeps the topic alive without
    // feeding the rate estimate (RC-12).
    void mark_idle(const uint32_t topic_id, const int64_t now_ns);
    TopicHealth query(const uint32_t topic_id) const;
    SystemStatus overall_status() const;
    void set_expected_hz(const uint32_t topic_id, const float32_t hz);

    void refresh_status(const uint32_t topic_id, const int64_t now_ns);

private:
    FixedVector<TopicHealth, kTopicCount> topics_;
};

} // namespace cuas
