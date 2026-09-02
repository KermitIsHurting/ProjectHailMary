// @file color_correct_engine.cpp
// @brief Implementation of per-channel BGR8 gain LUT.
#include "cuas_fusion/color_correct_engine.hpp"

namespace cuas {

namespace {

constexpr float32_t kMaxLevel = 255.0F;
constexpr float32_t kMaxGain  = 8.0F;

// A NaN gain survives both range checks in scale_clamp (comparisons are
// false) and reaches the undefined float->uint8 cast; out-of-range gains
// produce a solid-color image. Negated comparisons so NaN takes the
// identity fallback.
float32_t sanitize_gain(const float32_t gain)
{
    if (!(gain >= 0.0F) || !(gain <= kMaxGain)) {
        return 1.0F;
    }
    return gain;
}

uint8_t scale_clamp(const uint32_t input, const float32_t gain)
{
    const float32_t scaled = static_cast<float32_t>(input) * gain;
    if (scaled <= 0.0F) {
        return static_cast<uint8_t>(0);
    }
    if (scaled >= kMaxLevel) {
        return static_cast<uint8_t>(255);
    }
    return static_cast<uint8_t>(scaled);
}

}  // namespace

ColorCorrectEngine::ColorCorrectEngine()
: ColorCorrectEngine(kDefaultBlueGain, kDefaultGreenGain, kDefaultRedGain)
{
}

ColorCorrectEngine::ColorCorrectEngine(const float32_t blue_gain,
                                       const float32_t green_gain,
                                       const float32_t red_gain)
: blue_gain_(sanitize_gain(blue_gain)),
  green_gain_(sanitize_gain(green_gain)),
  red_gain_(sanitize_gain(red_gain)),
  blue_lut_{},
  green_lut_{},
  red_lut_{}
{
    build_lut(blue_gain_,  blue_lut_);
    build_lut(green_gain_, green_lut_);
    build_lut(red_gain_,   red_lut_);
}

void ColorCorrectEngine::build_lut(const float32_t gain, uint8_t (&lut)[kLutSize])
{
    for (uint32_t i = 0U; i < kLutSize; ++i) {
        lut[i] = scale_clamp(i, gain);
    }
}

void ColorCorrectEngine::apply_bgr(uint8_t* const data,
                                   const std::size_t pixel_count) const
{
    if (data == nullptr) {
        return;
    }
    for (std::size_t i = 0U; i < pixel_count; ++i) {
        const std::size_t idx = i * 3U;
        data[idx]        = blue_lut_ [data[idx]];
        data[idx + 1U]   = green_lut_[data[idx + 1U]];
        data[idx + 2U]   = red_lut_  [data[idx + 2U]];
    }
}

}  // namespace cuas
