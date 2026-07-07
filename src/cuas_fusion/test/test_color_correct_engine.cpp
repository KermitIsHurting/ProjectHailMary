// @file test_color_correct_engine.cpp
// @brief Unit tests for ColorCorrectEngine per-channel gain LUT.
#include "cuas_fusion/color_correct_engine.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <gtest/gtest.h>

#include <array>

namespace {

constexpr cuas::uint8_t kMid = 100U;

std::array<cuas::uint8_t, 3U> apply_one(
    const cuas::ColorCorrectEngine& engine,
    const cuas::uint8_t b,
    const cuas::uint8_t g,
    const cuas::uint8_t r)
{
    std::array<cuas::uint8_t, 3U> px{b, g, r};
    engine.apply_bgr(px.data(), 1U);
    return px;
}

}  // namespace

TEST(ColorCorrectEngineTest, DefaultGainsMatchSpec)
{
    const cuas::ColorCorrectEngine engine;
    EXPECT_FLOAT_EQ(engine.blue_gain(),  0.75F);
    EXPECT_FLOAT_EQ(engine.green_gain(), 1.00F);
    EXPECT_FLOAT_EQ(engine.red_gain(),   1.30F);
}

TEST(ColorCorrectEngineTest, AppliesPerChannelGain)
{
    const cuas::ColorCorrectEngine engine;
    const auto px = apply_one(engine, kMid, kMid, kMid);
    EXPECT_EQ(px[0], static_cast<cuas::uint8_t>(75));   // 100 * 0.75
    EXPECT_EQ(px[1], static_cast<cuas::uint8_t>(100));  // 100 * 1.00
    EXPECT_EQ(px[2], static_cast<cuas::uint8_t>(130));  // 100 * 1.30
}

TEST(ColorCorrectEngineTest, ClampsHighRedToMax)
{
    const cuas::ColorCorrectEngine engine;
    const auto px = apply_one(engine, 0U, 0U, 250U);
    // 250 * 1.30 = 325 → clamped to 255
    EXPECT_EQ(px[2], static_cast<cuas::uint8_t>(255));
}

TEST(ColorCorrectEngineTest, ClampsHighGreenToMaxOnUnityIsNoop)
{
    const cuas::ColorCorrectEngine engine;
    const auto px = apply_one(engine, 0U, 255U, 0U);
    EXPECT_EQ(px[1], static_cast<cuas::uint8_t>(255));
}

TEST(ColorCorrectEngineTest, BlueAttenuationAtMaxInput)
{
    const cuas::ColorCorrectEngine engine;
    const auto px = apply_one(engine, 200U, 0U, 0U);
    EXPECT_EQ(px[0], static_cast<cuas::uint8_t>(150));  // 200 * 0.75
}

TEST(ColorCorrectEngineTest, ZeroPixelsRemainZero)
{
    const cuas::ColorCorrectEngine engine;
    const auto px = apply_one(engine, 0U, 0U, 0U);
    EXPECT_EQ(px[0], 0U);
    EXPECT_EQ(px[1], 0U);
    EXPECT_EQ(px[2], 0U);
}

TEST(ColorCorrectEngineTest, NullDataIsSafe)
{
    const cuas::ColorCorrectEngine engine;
    engine.apply_bgr(nullptr, 0U);
    engine.apply_bgr(nullptr, 16U);
    SUCCEED();
}

TEST(ColorCorrectEngineTest, CustomGainsApplied)
{
    const cuas::ColorCorrectEngine engine(0.5F, 2.0F, 0.0F);
    const auto px = apply_one(engine, 100U, 100U, 100U);
    EXPECT_EQ(px[0], static_cast<cuas::uint8_t>(50));
    EXPECT_EQ(px[1], static_cast<cuas::uint8_t>(200));
    EXPECT_EQ(px[2], static_cast<cuas::uint8_t>(0));
}

TEST(ColorCorrectEngineTest, MultiplePixelsProcessedIndependently)
{
    const cuas::ColorCorrectEngine engine;
    std::array<cuas::uint8_t, 6U> buf{
        kMid, kMid, kMid,
        50U,  50U,  50U,
    };
    engine.apply_bgr(buf.data(), 2U);
    EXPECT_EQ(buf[0], static_cast<cuas::uint8_t>(75));
    EXPECT_EQ(buf[1], static_cast<cuas::uint8_t>(100));
    EXPECT_EQ(buf[2], static_cast<cuas::uint8_t>(130));
    EXPECT_EQ(buf[3], static_cast<cuas::uint8_t>(37));   // 50 * 0.75 = 37.5 → 37
    EXPECT_EQ(buf[4], static_cast<cuas::uint8_t>(50));
    EXPECT_EQ(buf[5], static_cast<cuas::uint8_t>(65));   // 50 * 1.30
}

TEST(ColorCorrectEngineTest, ZeroCountIsNoop)
{
    const cuas::ColorCorrectEngine engine;
    std::array<cuas::uint8_t, 3U> buf{10U, 20U, 30U};
    engine.apply_bgr(buf.data(), 0U);
    EXPECT_EQ(buf[0], 10U);
    EXPECT_EQ(buf[1], 20U);
    EXPECT_EQ(buf[2], 30U);
}
