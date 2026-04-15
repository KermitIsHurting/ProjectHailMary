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
    float32_t   last_recv_sec;
    float32_t   expected_hz;
    float32_t   measured_hz;
    TopicStatus status;
};

class HealthMonitor {
public:
    static constexpr float32_t kStaleThresholdSec = 0.5F;
    static constexpr float32_t kDeadThresholdSec  = 2.0F;
    static constexpr float32_t kEmaAlpha          = 0.1F;

    HealthMonitor();

    void update(const uint32_t topic_id, const float32_t now_sec);
    TopicHealth query(const uint32_t topic_id) const;
    SystemStatus overall_status() const;
    void set_expected_hz(const uint32_t topic_id, const float32_t hz);

    void refresh_status(const uint32_t topic_id, const float32_t now_sec);

private:
    FixedVector<TopicHealth, kTopicCount> topics_;
};

} // namespace cuas
