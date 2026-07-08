// test_eigen_types.cpp
// Unit tests for the Track.msg covariance wire packer (P3.1). The
// row-major upper-triangle ordering is the contract shared by the legacy
// and central trackers; the anchor test pins the layout with literal
// indices so a reordered packer cannot pass by mirroring its own loop.

#include "cuas_fusion/common/eigen_types.hpp"

#include <gtest/gtest.h>

#include <array>

namespace {

TEST(PackUpperTriangle6, LayoutAnchors)
{
    cuas::Matrix6d P;
    for (Eigen::Index i = 0; i < 6; ++i) {
        for (Eigen::Index j = 0; j < 6; ++j) {
            P(i, j) = static_cast<cuas::float64_t>((i * 6) + j);
        }
    }
    std::array<cuas::float64_t, 21> wire{};
    cuas::packUpperTriangle6(P, wire);

    // Row i of the upper triangle starts at offset 0, 6, 11, 15, 18, 20.
    EXPECT_DOUBLE_EQ(wire[0],  0.0);   // P(0,0)
    EXPECT_DOUBLE_EQ(wire[1],  1.0);   // P(0,1)
    EXPECT_DOUBLE_EQ(wire[5],  5.0);   // P(0,5)
    EXPECT_DOUBLE_EQ(wire[6],  7.0);   // P(1,1)
    EXPECT_DOUBLE_EQ(wire[7],  8.0);   // P(1,2)
    EXPECT_DOUBLE_EQ(wire[11], 14.0);  // P(2,2)
    EXPECT_DOUBLE_EQ(wire[15], 21.0);  // P(3,3)
    EXPECT_DOUBLE_EQ(wire[18], 28.0);  // P(4,4)
    EXPECT_DOUBLE_EQ(wire[19], 29.0);  // P(4,5)
    EXPECT_DOUBLE_EQ(wire[20], 35.0);  // P(5,5)
}

TEST(PackUpperTriangle6, SymmetricRoundTrip)
{
    cuas::Matrix6d P;
    for (Eigen::Index i = 0; i < 6; ++i) {
        for (Eigen::Index j = 0; j < 6; ++j) {
            const Eigen::Index lo = (i < j) ? i : j;
            const Eigen::Index hi = (i < j) ? j : i;
            P(i, j) = 1.0 + static_cast<cuas::float64_t>((lo * 6) + hi);
        }
    }
    std::array<cuas::float64_t, 21> wire{};
    cuas::packUpperTriangle6(P, wire);

    cuas::Matrix6d Q;
    std::size_t k = 0U;
    for (Eigen::Index i = 0; i < 6; ++i) {
        for (Eigen::Index j = i; j < 6; ++j) {
            Q(i, j) = wire[k];
            Q(j, i) = wire[k];
            ++k;
        }
    }
    EXPECT_EQ(k, wire.size());
    EXPECT_TRUE(Q.isApprox(P));
}

}  // namespace
