// @file color_correct_engine.hpp
// @brief Pure-math per-channel BGR8 gain correction with precomputed clamping LUTs.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"

#include <cstddef>

namespace cuas {

class ColorCorrectEngine {
public:
    static constexpr float32_t kDefaultBlueGain  = 0.75F;
    static constexpr float32_t kDefaultGreenGain = 1.00F;
    static constexpr float32_t kDefaultRedGain   = 1.30F;
    static constexpr uint32_t  kLutSize          = 256U;

    ColorCorrectEngine();
    ColorCorrectEngine(const float32_t blue_gain,
                       const float32_t green_gain,
                       const float32_t red_gain);

    // Apply per-channel gains in place to an interleaved BGR8 buffer.
    // pixel_count is the number of 3-byte BGR triplets in data.
    // Safe to call with data == nullptr (no-op).
    void apply_bgr(uint8_t* const data, const std::size_t pixel_count) const;

    float32_t blue_gain()  const { return blue_gain_; }
    float32_t green_gain() const { return green_gain_; }
    float32_t red_gain()   const { return red_gain_; }

private:
    static void build_lut(const float32_t gain, uint8_t (&lut)[kLutSize]);

    float32_t blue_gain_;
    float32_t green_gain_;
    float32_t red_gain_;
    uint8_t   blue_lut_[kLutSize];
    uint8_t   green_lut_[kLutSize];
    uint8_t   red_lut_[kLutSize];
};

}  // namespace cuas
