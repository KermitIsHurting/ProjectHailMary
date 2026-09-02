// test_kinematic_predictor.cpp
// Unit tests for the CV forward propagator. The state builder must carry the
// track's own velocity vector: the previous speed-along-bearing builder sent
// every forecast radially outward regardless of true heading, so an
// approaching target was predicted to recede.

#include "cuas_fusion/prediction/kinematic_predictor.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <limits>

namespace {

using cuas::KinematicPredictor;

KinematicPredictor::TrajectoryResult propagate(const Eigen::VectorXd& state,
                                               double dt, int steps)
{
    KinematicPredictor predictor;
    const Eigen::MatrixXd P = KinematicPredictor::build_initial_covariance_6d();
    const Eigen::MatrixXd F = KinematicPredictor::build_transition_matrix_6d(dt);
    const Eigen::MatrixXd Q = KinematicPredictor::build_process_noise_6d(dt, 0.25);
    return predictor.propagateForward(state, P, F, Q, dt, steps);
}

// RC-28: the per-track horizon decides the step count; past the 64-step
// buffer the step coarsens so the last waypoint still lands on the horizon.
TEST(KinematicPredictor, StepPlanFollowsHorizon)
{
    const auto p3 = KinematicPredictor::planSteps(3.0, 0.1, 5.0);
    EXPECT_EQ(p3.n_steps, 30);
    EXPECT_NEAR(p3.step_dt, 0.1, 1e-9);
    EXPECT_NEAR(p3.n_steps * p3.step_dt, 3.0, 1e-9);

    const auto p10 = KinematicPredictor::planSteps(10.0, 0.1, 5.0);
    EXPECT_EQ(p10.n_steps, static_cast<int32_t>(cuas::kMaxTrajectorySteps));
    EXPECT_NEAR(p10.n_steps * p10.step_dt, 10.0, 1e-9);

    const auto fallback = KinematicPredictor::planSteps(0.0, 0.1, 5.0);
    EXPECT_NEAR(fallback.n_steps * fallback.step_dt, 5.0, 1e-9);
    const auto nan_h = KinematicPredictor::planSteps(
        std::numeric_limits<double>::quiet_NaN(), 0.1, 5.0);
    EXPECT_NEAR(nan_h.n_steps * nan_h.step_dt, 5.0, 1e-9);
}

TEST(KinematicPredictor, StateBuilderCarriesGivenVelocity)
{
    // Approaching target at (5, 0): vx must be -1. The old builder derived
    // +1 (outward along the bearing) from the same track.
    const Eigen::VectorXd s =
        KinematicPredictor::build_state_from_position_velocity(
            5.0, 0.0, 0.0, -1.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(s(0), 5.0);
    EXPECT_DOUBLE_EQ(s(3), -1.0);
    EXPECT_DOUBLE_EQ(s(4), 0.0);
    EXPECT_DOUBLE_EQ(s(5), 0.0);
}

TEST(KinematicPredictor, ApproachingTrackMovesTowardSensor)
{
    const Eigen::VectorXd s =
        KinematicPredictor::build_state_from_position_velocity(
            5.0, 0.0, 0.0, -1.0, 0.0, 0.0);
    const auto traj = propagate(s, 0.1, 10);
    ASSERT_EQ(traj.positions.size(), 10U);
    EXPECT_NEAR(traj.positions.back().x(), 4.0, 1e-9);

    // Range must fall every step; outward propagation would raise it.
    double prev = 5.0;
    for (uint32_t i = 0U; i < traj.positions.size(); ++i) {
        const double r = traj.positions[i].norm();
        EXPECT_LT(r, prev);
        prev = r;
    }
}

TEST(KinematicPredictor, TangentialVelocityIsNotRotatedOntoBearing)
{
    // Target at (0, 5) crossing in +x: y stays 5, x advances.
    const Eigen::VectorXd s =
        KinematicPredictor::build_state_from_position_velocity(
            0.0, 5.0, 0.0, 2.0, 0.0, 0.0);
    const auto traj = propagate(s, 0.1, 5);
    ASSERT_EQ(traj.positions.size(), 5U);
    EXPECT_NEAR(traj.positions.back().x(), 1.0, 1e-9);
    EXPECT_NEAR(traj.positions.back().y(), 5.0, 1e-9);
}

TEST(KinematicPredictor, VerticalVelocityPropagates)
{
    const Eigen::VectorXd s =
        KinematicPredictor::build_state_from_position_velocity(
            3.0, 4.0, 0.0, 0.0, 0.0, 1.5);
    const auto traj = propagate(s, 0.2, 5);
    ASSERT_EQ(traj.positions.size(), 5U);
    EXPECT_NEAR(traj.positions.back().z(), 1.5, 1e-9);
    EXPECT_GT(traj.final_elevation_deg, 0.0);
}

TEST(KinematicPredictor, StepCountClampsToBuffer)
{
    const Eigen::VectorXd s =
        KinematicPredictor::build_state_from_position_velocity(
            1.0, 1.0, 0.0, 0.5, 0.0, 0.0);
    const auto traj = propagate(s, 0.1, 1000);
    EXPECT_EQ(traj.positions.size(), cuas::kMaxTrajectorySteps);
    EXPECT_EQ(traj.timestamps_sec.size(), cuas::kMaxTrajectorySteps);
}

TEST(KinematicPredictor, UncertaintyGrowsAndIsCapped)
{
    const Eigen::VectorXd s =
        KinematicPredictor::build_state_from_position_velocity(
            2.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    const auto traj = propagate(s, 0.5, 64);
    ASSERT_FALSE(traj.uncertainty_radii_m.empty());
    const double cap = static_cast<double>(cuas::kMaxUncertaintyRadiusM);
    for (uint32_t i = 1U; i < traj.uncertainty_radii_m.size(); ++i) {
        EXPECT_GE(traj.uncertainty_radii_m[i], traj.uncertainty_radii_m[i - 1U]);
        EXPECT_LE(traj.uncertainty_radii_m[i], cap);
    }
}

TEST(KinematicPredictor, TransitionAndNoiseStructure)
{
    const double dt = 0.1;
    const Eigen::MatrixXd F = KinematicPredictor::build_transition_matrix_6d(dt);
    EXPECT_DOUBLE_EQ(F(0, 3), dt);
    EXPECT_DOUBLE_EQ(F(1, 4), dt);
    EXPECT_DOUBLE_EQ(F(2, 5), dt);

    const Eigen::MatrixXd Q = KinematicPredictor::build_process_noise_6d(dt, 0.25);
    EXPECT_TRUE(Q.isApprox(Q.transpose()));
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q);
    EXPECT_GE(es.eigenvalues().minCoeff(), -1e-15);
}

} // namespace
