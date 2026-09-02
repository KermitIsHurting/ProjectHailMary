// test_trt_topk.cpp
// Unit tests for the pre-NMS top-K candidate keeper (audit B4 / RC-19).

#include "cuas_fusion/inference/topk_boxes.hpp"

#include <gtest/gtest.h>
#include <algorithm>

namespace {

cuas::BoundingBox box(float conf, int32_t id)
{
    cuas::BoundingBox b{};
    b.x = 0.0F;
    b.y = 0.0F;
    b.w = 10.0F;
    b.h = 10.0F;
    b.confidence = conf;
    b.class_id   = id;
    return b;
}

} // namespace

TEST(TopKBoxes, KeepsHighestConfidenceRegardlessOfArrivalOrder)
{
    // 200 candidates in anchor order, confidence rising with the index:
    // the K best are the LAST K offered, which the old first-K cap dropped.
    cuas::TopKBoxes<128U> keeper;
    for (int32_t i = 0; i < 200; ++i) {
        keeper.offer(box(0.25F + 0.003F * static_cast<float>(i), i));
    }
    ASSERT_EQ(keeper.size(), 128U);
    float min_kept = 1.0F;
    for (uint32_t i = 0U; i < keeper.size(); ++i) {
        min_kept = std::min(min_kept, keeper.boxes()[i].confidence);
        EXPECT_GE(keeper.boxes()[i].class_id, 72) << "a dropped low-score box survived";
    }
    EXPECT_NEAR(min_kept, 0.25F + 0.003F * 72.0F, 1e-6F);
}

TEST(TopKBoxes, BelowCapacityKeepsEverythingAndLateStrongBoxDisplacesWeakest)
{
    cuas::TopKBoxes<4U> keeper;
    keeper.offer(box(0.5F, 1));
    keeper.offer(box(0.3F, 2));
    keeper.offer(box(0.9F, 3));
    EXPECT_EQ(keeper.size(), 3U);
    keeper.offer(box(0.4F, 4));
    EXPECT_EQ(keeper.size(), 4U);
    keeper.offer(box(0.2F, 5));   // weaker than every kept box: ignored
    keeper.offer(box(0.6F, 6));   // displaces 0.3 (id 2)
    ASSERT_EQ(keeper.size(), 4U);
    bool saw2 = false;
    bool saw6 = false;
    for (uint32_t i = 0U; i < keeper.size(); ++i) {
        saw2 = saw2 || (keeper.boxes()[i].class_id == 2);
        saw6 = saw6 || (keeper.boxes()[i].class_id == 6);
    }
    EXPECT_FALSE(saw2);
    EXPECT_TRUE(saw6);
}
