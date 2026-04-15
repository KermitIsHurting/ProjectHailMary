// @file clutter_map.cpp
// @brief ClutterMap implementation: cell hit counting and normalized occupancy query.
#include "cuas_fusion/clutter_map.hpp"

namespace cuas {

ClutterMap::ClutterMap()
: grid_{}
, frames_seen_(0U)
, learn_frames_(kLearnFrames)
, threshold_(kThreshold)
, learned_(false)
{
    reset();
}

void ClutterMap::reset()
{
    for (uint32_t col = 0U; col < kGridW; ++col) {
        for (uint32_t row = 0U; row < kGridH; ++row) {
            grid_[col][row] = 0.0F;
        }
    }
    frames_seen_ = 0U;
    learned_     = false;
}

void ClutterMap::set_learn_frames(const uint32_t frames)
{
    if (frames == 0U) {
        learn_frames_ = 1U;
    } else {
        learn_frames_ = frames;
    }
}

void ClutterMap::set_threshold(const float32_t threshold)
{
    threshold_ = threshold;
}

bool ClutterMap::cell_index(const float32_t x_m, const float32_t y_m,
                            uint32_t & col, uint32_t & row) const
{
    if (x_m < kOriginX_m) {
        return false;
    }
    if (y_m < kOriginY_m) {
        return false;
    }
    const float32_t col_f = (x_m - kOriginX_m) / kCellSize_m;
    const float32_t row_f = (y_m - kOriginY_m) / kCellSize_m;
    if (col_f < 0.0F) {
        return false;
    }
    if (row_f < 0.0F) {
        return false;
    }
    const uint32_t c = static_cast<uint32_t>(col_f);
    const uint32_t r = static_cast<uint32_t>(row_f);
    if (c >= kGridW) {
        return false;
    }
    if (r >= kGridH) {
        return false;
    }
    col = c;
    row = r;
    return true;
}

void ClutterMap::add_frame(const FixedVector<float32_t, kClutterMapMaxPoints> & xs,
                           const FixedVector<float32_t, kClutterMapMaxPoints> & ys)
{
    if (learned_) {
        return;
    }

    uint32_t n = ys.size();
    if (xs.size() < ys.size()) {
        n = xs.size();
    }
    for (uint32_t i = 0U; i < n; ++i) {
        uint32_t col = 0U;
        uint32_t row = 0U;
        if (cell_index(xs[i], ys[i], col, row)) {
            grid_[col][row] += 1.0F;
        }
    }

    ++frames_seen_;
    if (frames_seen_ >= learn_frames_) {
        const float32_t divisor = static_cast<float32_t>(learn_frames_);
        for (uint32_t col = 0U; col < kGridW; ++col) {
            for (uint32_t row = 0U; row < kGridH; ++row) {
                grid_[col][row] /= divisor;
            }
        }
        learned_ = true;
    }
}

bool ClutterMap::is_clutter(const float32_t x_m, const float32_t y_m) const
{
    if (!learned_) {
        return false;
    }
    uint32_t col = 0U;
    uint32_t row = 0U;
    if (!cell_index(x_m, y_m, col, row)) {
        return false;
    }
    return grid_[col][row] >= threshold_;
}

bool ClutterMap::is_learned() const
{
    return learned_;
}

uint32_t ClutterMap::frame_count() const
{
    return frames_seen_;
}

float32_t ClutterMap::occupancy_ratio() const
{
    if (!learned_) {
        return 0.0F;
    }
    uint32_t count = 0U;
    for (uint32_t col = 0U; col < kGridW; ++col) {
        for (uint32_t row = 0U; row < kGridH; ++row) {
            if (grid_[col][row] >= kThreshold) {
                ++count;
            }
        }
    }
    const float32_t total = static_cast<float32_t>(kGridW * kGridH);
    return static_cast<float32_t>(count) / total;
}

} // namespace cuas
