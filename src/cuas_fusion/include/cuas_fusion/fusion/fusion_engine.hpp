// @file fusion_engine.hpp
// @brief Radar detection to YOLO box projector and associator.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <array>

namespace cuas {

class FusionEngine {
public:
    FusionEngine() = default;

    bool init(const ExtrinsicTransform& extrinsic);

    bool projectAndAssociate(
        const FixedVector<RadarDetection, TRACK_MAX_TRACKS>& radar_pts,
        const FixedVector<BoundingBox, 128U>& yolo_boxes,
        FixedVector<FusedDetection, TRACK_MAX_TRACKS>& fused_out);

private:
    static constexpr float32_t kEmaAlpha = 0.35F;
    // Association gate: a fused position farther than this from every valid
    // same-class state starts a fresh smoothing state instead of blending.
    static constexpr float32_t kEmaGateM = 2.0F;
    // States not refreshed within this window (measurement time) are evicted,
    // so a returning target cannot blend with its minutes-old ghost.
    static constexpr int64_t   kEmaTimeoutNs = 1'000'000'000LL;

    struct EmaState {
        float32_t x   = 0.0F;
        float32_t y   = 0.0F;
        float32_t z   = 0.0F;
        float32_t vel = 0.0F;
        int32_t   class_id       = -1;
        int64_t   last_update_ns = 0;
        bool      valid          = false;
    };

    EmaState* associateEma(int32_t class_id, float32_t x, float32_t y,
                           float32_t z, int64_t now_ns);

    // SE(3) baked at init: p_cam = rot_ · p_radar + trans_ (row-major R).
    std::array<float32_t, 9> rot_{};
    std::array<float32_t, 3> trans_{};
    bool               initialized_ = false;
    uint32_t           miss_count_  = 0U;
    // EMA state keyed by association identity (nearest gate match), not by
    // class: two same-class targets each hold their own state (A1.8).
    std::array<EmaState, TRACK_MAX_TRACKS> ema_states_{};
};

} // namespace cuas
