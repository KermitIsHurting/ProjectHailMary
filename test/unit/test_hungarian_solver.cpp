// test_hungarian_solver.cpp
// Unit tests for the Jonker-Volgenant assignment solver: optimality on
// small matrices, rectangular handling, gating, and capacity rejection.

#include "cuas_fusion/tracking/hungarian_solver.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace {

using Assignment = cuas::FixedVector<int32_t, cuas::TRACK_MAX_TRACKS>;

TEST(HungarianSolver, IdentityCostAssignsDiagonal)
{
    cuas::HungarianSolver solver;
    Eigen::MatrixXd cost(3, 3);
    cost << 0.0, 1.0, 1.0,
            1.0, 0.0, 1.0,
            1.0, 1.0, 0.0;
    Assignment assignment;
    Assignment unassigned;
    ASSERT_TRUE(solver.solve(cost, assignment, unassigned));
    ASSERT_EQ(assignment.size(), 3U);
    EXPECT_EQ(assignment[0], 0);
    EXPECT_EQ(assignment[1], 1);
    EXPECT_EQ(assignment[2], 2);
    EXPECT_EQ(unassigned.size(), 0U);
}

TEST(HungarianSolver, PicksGloballyMinimalAssignment)
{
    cuas::HungarianSolver solver;
    // Greedy row-wise would pick (0,0)=1 then (1,1)=4 -> total 5.
    // Optimal is (0,1)=2 + (1,0)=2 -> total 4.
    Eigen::MatrixXd cost(2, 2);
    cost << 1.0, 2.0,
            2.0, 4.0;
    Assignment assignment;
    Assignment unassigned;
    ASSERT_TRUE(solver.solve(cost, assignment, unassigned));
    ASSERT_EQ(assignment.size(), 2U);
    EXPECT_EQ(assignment[0], 1);
    EXPECT_EQ(assignment[1], 0);
}

TEST(HungarianSolver, RectangularMoreDetectionsThanTracks)
{
    cuas::HungarianSolver solver;
    Eigen::MatrixXd cost(1, 3);
    cost << 5.0, 1.0, 5.0;
    Assignment assignment;
    Assignment unassigned;
    ASSERT_TRUE(solver.solve(cost, assignment, unassigned));
    ASSERT_EQ(assignment.size(), 1U);
    EXPECT_EQ(assignment[0], 1);
    // The two unclaimed detections must be reported.
    ASSERT_EQ(unassigned.size(), 2U);
    EXPECT_EQ(unassigned[0], 0);
    EXPECT_EQ(unassigned[1], 2);
}

TEST(HungarianSolver, GatedPairsAreDropped)
{
    cuas::HungarianSolver solver;
    Eigen::MatrixXd cost(2, 2);
    cost << 1.0, cuas::HungarianSolver::kLargeCost,
            cuas::HungarianSolver::kLargeCost, cuas::HungarianSolver::kLargeCost;
    Assignment assignment;
    Assignment unassigned;
    ASSERT_TRUE(solver.solve(cost, assignment, unassigned));
    ASSERT_EQ(assignment.size(), 2U);
    EXPECT_EQ(assignment[0], 0);
    EXPECT_EQ(assignment[1], -1);  // only gated pairs available -> unassigned
    ASSERT_EQ(unassigned.size(), 1U);
    EXPECT_EQ(unassigned[0], 1);
}

TEST(HungarianSolver, RejectsOversizedInput)
{
    cuas::HungarianSolver solver;
    const auto n = static_cast<Eigen::Index>(cuas::TRACK_MAX_TRACKS) + 1;
    const Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(n, n);
    Assignment assignment;
    Assignment unassigned;
    EXPECT_FALSE(solver.solve(cost, assignment, unassigned));
}

TEST(HungarianSolver, NonFiniteCostRejectedNotHung)
{
    cuas::HungarianSolver solver;
    Eigen::MatrixXd cost(2, 2);
    cost << 1.0, 2.0,
            std::nan(""), 4.0;
    Assignment assignment;
    Assignment unassigned;
    // Before the allFinite() guard this input made the augmenting-path loop
    // spin forever (every NaN comparison is false, j1 never advances).
    EXPECT_FALSE(solver.solve(cost, assignment, unassigned));

    cost(1, 0) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(solver.solve(cost, assignment, unassigned));
}

TEST(HungarianSolver, EmptyMatrixYieldsEmptyAssignment)
{
    cuas::HungarianSolver solver;
    const Eigen::MatrixXd cost(0, 0);
    Assignment assignment;
    Assignment unassigned;
    ASSERT_TRUE(solver.solve(cost, assignment, unassigned));
    EXPECT_EQ(assignment.size(), 0U);
    EXPECT_EQ(unassigned.size(), 0U);
}

}  // namespace
