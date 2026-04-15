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

static constexpr uint8_t kStatusOk     = 0U;
static constexpr uint8_t kStatusStale  = 1U;
static constexpr uint8_t kStatusDead   = 2U;

static constexpr uint8_t kOverallNominal  = 0U;
static constexpr uint8_t kOverallDegraded = 1U;
static constexpr uint8_t kOverallFailed   = 2U;

struct TopicHealth {
    float32_t last_recv_sec;
    float32_t expected_hz;
    float32_t measured_hz;
    uint8_t   status;
};

class HealthMonitor {
public:
    static constexpr float32_t kStaleThresholdSec = 0.5F;
    static constexpr float32_t kDeadThresholdSec  = 2.0F;
    static constexpr float32_t kEmaAlpha          = 0.1F;

    HealthMonitor();

    void update(const uint32_t topic_id, const float32_t now_sec);
    TopicHealth query(const uint32_t topic_id) const;
    uint8_t overall_status() const;
    void set_expected_hz(const uint32_t topic_id, const float32_t hz);

    void refresh_status(const uint32_t topic_id, const float32_t now_sec);

private:
    FixedVector<TopicHealth, kTopicCount> topics_;
};

} // namespace cuas
