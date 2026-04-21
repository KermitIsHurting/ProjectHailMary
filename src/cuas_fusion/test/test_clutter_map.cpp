// @file test_clutter_map.cpp
// @brief Unit tests for ClutterMap occupancy grid logic.
#include "cuas_fusion/clutter_map.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"

#include <gtest/gtest.h>

namespace {

void feed_frames_at(cuas::ClutterMap & map,
                    const cuas::float32_t x_m,
                    const cuas::float32_t y_m,
                    const cuas::uint32_t count)
{
    cuas::FixedVector<cuas::float32_t, cuas::kClutterMapMaxPoints> xs;
    cuas::FixedVector<cuas::float32_t, cuas::kClutterMapMaxPoints> ys;
    (void)xs.push_back(x_m);
    (void)ys.push_back(y_m);
    for (cuas::uint32_t i = 0U; i < count; ++i) {
        map.add_frame(xs, ys);
    }
}

void feed_empty_frames(cuas::ClutterMap & map, const cuas::uint32_t count)
{
    cuas::FixedVector<cuas::float32_t, cuas::kClutterMapMaxPoints> xs;
    cuas::FixedVector<cuas::float32_t, cuas::kClutterMapMaxPoints> ys;
    for (cuas::uint32_t i = 0U; i < count; ++i) {
        map.add_frame(xs, ys);
    }
}

} // namespace

TEST(ClutterMapTest, NotLearnedBeforeFrames)
{
    cuas::ClutterMap map;
    feed_empty_frames(map, cuas::ClutterMap::kLearnFrames - 1U);
    EXPECT_FALSE(map.is_learned());
}

TEST(ClutterMapTest, LearnedAfterFrames)
{
    cuas::ClutterMap map;
    feed_empty_frames(map, cuas::ClutterMap::kLearnFrames);
    EXPECT_TRUE(map.is_learned());
}

TEST(ClutterMapTest, ClutterDetected)
{
    cuas::ClutterMap map;
    feed_frames_at(map, 0.0F, 0.0F, cuas::ClutterMap::kLearnFrames);
    ASSERT_TRUE(map.is_learned());
    EXPECT_TRUE(map.is_clutter(0.0F, 0.0F));
}

TEST(ClutterMapTest, NoClutterEmptyCell)
{
    cuas::ClutterMap map;
    feed_frames_at(map, 0.0F, 0.0F, cuas::ClutterMap::kLearnFrames);
    ASSERT_TRUE(map.is_learned());
    EXPECT_FALSE(map.is_clutter(4.0F, 4.0F));
}

TEST(ClutterMapTest, OutsideGridReturnsFalse)
{
    cuas::ClutterMap map;
    feed_frames_at(map, 0.0F, 0.0F, cuas::ClutterMap::kLearnFrames);
    ASSERT_TRUE(map.is_learned());
    EXPECT_FALSE(map.is_clutter(99.0F, 99.0F));
}

TEST(ClutterMapTest, ResetClearsLearned)
{
    cuas::ClutterMap map;
    feed_empty_frames(map, cuas::ClutterMap::kLearnFrames);
    ASSERT_TRUE(map.is_learned());
    map.reset();
    EXPECT_FALSE(map.is_learned());
}

TEST(ClutterMapTest, OccupancyRatioZeroBeforeLearn)
{
    cuas::ClutterMap map;
    feed_empty_frames(map, cuas::ClutterMap::kLearnFrames - 1U);
    EXPECT_FLOAT_EQ(map.occupancy_ratio(), 0.0F);
}
