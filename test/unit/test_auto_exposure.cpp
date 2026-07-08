// test_auto_exposure.cpp
// Unit tests for the two-stage AE controller: exposure carries brightness
// up to the motion-blur ceiling, gain only past it; gain sheds before
// exposure on the way down; deadband, NaN, clamp, and rounding-stall
// behavior are pinned so the loop can neither oscillate nor wedge.

#include "cuas_fusion/drivers/auto_exposure.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using cuas::AeCommand;
using cuas::AeParams;
using cuas::nextAeCommand;

AeParams defaultParams()
{
    AeParams p;
    p.target_mean   = 110.0F;
    p.deadband      = 10.0F;
    p.max_step_frac = 0.25F;
    p.limits.exposure_min = 2;
    p.limits.exposure_max = 8000;
    p.limits.gain_min     = 100;
    p.limits.gain_max     = 1200;
    return p;
}

TEST(AutoExposure, RaisesExposureFirstWhenDark)
{
    const AeCommand c = nextAeCommand(700, 100, 40.0F, defaultParams());
    EXPECT_TRUE(c.changed);
    EXPECT_EQ(c.exposure, 875);  // 700 * (1 + 0.25), step clamped
    EXPECT_EQ(c.gain, 100);      // gain untouched below the ceiling
}

TEST(AutoExposure, GainOnlyAfterExposureCeiling)
{
    const AeCommand c = nextAeCommand(8000, 100, 40.0F, defaultParams());
    EXPECT_TRUE(c.changed);
    EXPECT_EQ(c.exposure, 8000);
    EXPECT_EQ(c.gain, 125);  // 100 * 1.25
}

TEST(AutoExposure, ShedsGainBeforeExposureWhenBright)
{
    const AeCommand c = nextAeCommand(3000, 400, 200.0F, defaultParams());
    EXPECT_TRUE(c.changed);
    EXPECT_EQ(c.exposure, 3000);
    EXPECT_EQ(c.gain, 300);  // 400 * (1 - 0.25)
}

TEST(AutoExposure, LowersExposureWhenBrightAtMinimumGain)
{
    const AeCommand c = nextAeCommand(3000, 100, 220.0F, defaultParams());
    EXPECT_TRUE(c.changed);
    EXPECT_EQ(c.exposure, 2250);  // 3000 * 0.75
    EXPECT_EQ(c.gain, 100);
}

TEST(AutoExposure, DeadbandHolds)
{
    const AeCommand c = nextAeCommand(700, 100, 105.0F, defaultParams());
    EXPECT_FALSE(c.changed);
    EXPECT_EQ(c.exposure, 700);
    EXPECT_EQ(c.gain, 100);
}

TEST(AutoExposure, NonFiniteMeanHolds)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const AeCommand c = nextAeCommand(700, 100, nan, defaultParams());
    EXPECT_FALSE(c.changed);
    EXPECT_EQ(c.exposure, 700);
    EXPECT_EQ(c.gain, 100);
}

TEST(AutoExposure, ClampsStepToCeiling)
{
    const AeCommand c = nextAeCommand(7900, 100, 40.0F, defaultParams());
    EXPECT_EQ(c.exposure, 8000);  // 7900 * 1.25 clamped to the ceiling
    EXPECT_EQ(c.gain, 100);
}

TEST(AutoExposure, PullsExternallySetExposureIntoLimits)
{
    // Manually-set 20000 exceeds the motion-blur ceiling: enforced even
    // when the brightness itself is inside the deadband.
    const AeCommand c = nextAeCommand(20000, 100, 110.0F, defaultParams());
    EXPECT_TRUE(c.changed);
    EXPECT_EQ(c.exposure, 8000);
    EXPECT_EQ(c.gain, 100);
}

TEST(AutoExposure, MinimumStepEscapesRoundingStall)
{
    // err just outside the deadband: 4 * 1.1 rounds back to 4; the
    // controller must still move by one count.
    const AeCommand c = nextAeCommand(4, 100, 99.0F, defaultParams());
    EXPECT_TRUE(c.changed);
    EXPECT_EQ(c.exposure, 5);
}

TEST(AutoExposure, SaturatedDarkSceneMakesNoChange)
{
    const AeCommand c = nextAeCommand(8000, 1200, 40.0F, defaultParams());
    EXPECT_FALSE(c.changed);
    EXPECT_EQ(c.exposure, 8000);
    EXPECT_EQ(c.gain, 1200);
}

}  // namespace
