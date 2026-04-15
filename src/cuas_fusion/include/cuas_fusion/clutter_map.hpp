// @file clutter_map.hpp
// @brief Pure-math static occupancy grid for persistent radar clutter rejection.
#pragma once

#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

static constexpr uint32_t  kClutterMapMaxPoints = 128U;

class ClutterMap {
public:
    static constexpr uint32_t  kGridW        = 40U;
    static constexpr uint32_t  kGridH        = 40U;
    static constexpr float32_t kCellSize_m   = 0.25F;
    static constexpr float32_t kOriginX_m    = -5.0F;
    static constexpr float32_t kOriginY_m    = -5.0F;
    static constexpr uint32_t  kLearnFrames  = 200U;
    static constexpr float32_t kThreshold    = 0.6F;

    ClutterMap();

    void add_frame(const FixedVector<float32_t, kClutterMapMaxPoints> & xs,
                   const FixedVector<float32_t, kClutterMapMaxPoints> & ys);
    bool is_clutter(const float32_t x_m, const float32_t y_m) const;
    bool is_learned() const;
    void reset();

    void set_learn_frames(const uint32_t frames);
    void set_threshold(const float32_t threshold);

    uint32_t frames_seen() const { return frames_seen_; }
    uint32_t learn_frames() const { return learn_frames_; }

    uint32_t  frame_count() const;
    float32_t occupancy_ratio() const;

private:
    bool cell_index(const float32_t x_m, const float32_t y_m,
                    uint32_t & col, uint32_t & row) const;

    float32_t grid_[kGridW][kGridH];
    uint32_t  frames_seen_;
    uint32_t  learn_frames_;
    float32_t threshold_;
    bool      learned_;
};

} // namespace cuas
