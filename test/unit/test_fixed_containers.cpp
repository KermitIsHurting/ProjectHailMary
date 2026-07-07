// test_fixed_containers.cpp
// Unit tests for FixedVector / FixedMap: capacity enforcement, overflow
// reporting, value-initialization guarantees, and the clamped accessors
// that keep out-of-contract use inside the array (MISRA 11.6.2 defense).

#include "cuas_fusion/common/fixed_containers.hpp"

#include <gtest/gtest.h>

namespace {

TEST(FixedVector, PushBackReportsOverflow)
{
    cuas::FixedVector<int, 3U> v;
    EXPECT_TRUE(v.push_back(1));
    EXPECT_TRUE(v.push_back(2));
    EXPECT_TRUE(v.push_back(3));
    EXPECT_TRUE(v.full());
    EXPECT_FALSE(v.push_back(4));
    EXPECT_EQ(v.size(), 3U);
    EXPECT_EQ(v[2], 3);
}

TEST(FixedVector, ElementsValueInitialized)
{
    cuas::FixedVector<int, 4U> v;
    ASSERT_TRUE(v.resize(4U));
    for (uint32_t i = 0U; i < v.size(); ++i) {
        EXPECT_EQ(v[i], 0);
    }
}

TEST(FixedVector, ResizeGrowthValueInitializesExposedSlots)
{
    cuas::FixedVector<int, 4U> v;
    ASSERT_TRUE(v.push_back(7));
    v.clear();
    // Growing over the slot that previously held 7 must re-initialize it.
    ASSERT_TRUE(v.resize(2U));
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 0);
}

TEST(FixedVector, ResizeRejectsOverCapacity)
{
    cuas::FixedVector<int, 2U> v;
    EXPECT_FALSE(v.resize(3U));
    EXPECT_EQ(v.size(), 0U);
    EXPECT_TRUE(v.resize(2U));
    EXPECT_EQ(v.size(), 2U);
}

TEST(FixedVector, BackOnEmptyDoesNotIndexOutOfBounds)
{
    cuas::FixedVector<int, 2U> v;
    // Out-of-contract call: must stay inside the array (slot 0) rather than
    // wrapping size_-1U to 0xFFFFFFFF. Value-init makes the read defined.
    EXPECT_EQ(v.back(), 0);
    ASSERT_TRUE(v.push_back(42));
    EXPECT_EQ(v.back(), 42);
}

TEST(FixedVector, IndexClampedToCapacity)
{
    cuas::FixedVector<int, 2U> v;
    ASSERT_TRUE(v.push_back(1));
    ASSERT_TRUE(v.push_back(2));
    // Out-of-contract index: clamped to the last slot, never past the array.
    EXPECT_EQ(v[999U], 2);
}

TEST(FixedMap, InsertFindEraseRoundTrip)
{
    cuas::FixedMap<int, int, 2U> m;
    EXPECT_TRUE(m.insert_or_assign(1, 10));
    EXPECT_TRUE(m.insert_or_assign(2, 20));
    EXPECT_FALSE(m.insert_or_assign(3, 30));  // full
    EXPECT_TRUE(m.insert_or_assign(1, 11));   // assign to existing still works
    ASSERT_NE(m.find(1), nullptr);
    EXPECT_EQ(*m.find(1), 11);
    EXPECT_TRUE(m.erase(1));
    EXPECT_FALSE(m.erase(1));
    EXPECT_EQ(m.find(1), nullptr);
    EXPECT_EQ(m.size(), 1U);
}

TEST(FixedMap, EraseIfRemovesMatching)
{
    cuas::FixedMap<int, int, 4U> m;
    ASSERT_TRUE(m.insert_or_assign(1, 1));
    ASSERT_TRUE(m.insert_or_assign(2, 2));
    ASSERT_TRUE(m.insert_or_assign(3, 3));
    m.erase_if([](const int& key, const int&) { return (key % 2) == 1; });
    EXPECT_EQ(m.size(), 1U);
    EXPECT_NE(m.find(2), nullptr);
}

}  // namespace
