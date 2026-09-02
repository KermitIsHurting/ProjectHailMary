// test_imm_filter.cpp
// Unit tests for the IMM filter: weight normalization, blended-state
// consistency, and straight-line tracking behavior.

#include "cuas_fusion/estimation/imm_filter.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {

Eigen::VectorXd make_state(double px, double py, double pz,
                           double vx, double vy, double vz)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    x << px, py, pz, vx, vy, vz;
    return x;
}

class ImmFilterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        imm_.init(make_state(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
                  Eigen::MatrixXd::Identity(6, 6));
    }
    cuas::ImmFilter imm_;
    const Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(3, 3) * 0.01;
};

TEST_F(ImmFilterTest, InitPropagatesToAllModels)
{
    for (uint32_t m = 0U; m < 3U; ++m) {
        const Eigen::VectorXd x = imm_.getModelState(m);
        ASSERT_GE(x.size(), 6);
        EXPECT_NEAR(x(3), 1.0, 1e-12) << "model " << m;
    }
}

TEST_F(ImmFilterTest, WeightsAlwaysSumToOne)
{
    for (int k = 0; k < 20; ++k) {
        const double t = 0.05 * static_cast<double>(k + 1);
        imm_.predict(0.05);
        const Eigen::VectorXd z =
            (Eigen::VectorXd(3) << t, 0.0, 0.0).finished();
        imm_.update(z, R_);

        const auto mu = imm_.getModelWeights();
        EXPECT_NEAR(mu[0] + mu[1] + mu[2], 1.0, 1e-9);
        EXPECT_GE(mu[0], 0.0);
        EXPECT_GE(mu[1], 0.0);
        EXPECT_GE(mu[2], 0.0);
    }
}

TEST_F(ImmFilterTest, TracksStraightLineMotion)
{
    // Target moves +x at 1 m/s; measurements are exact.
    for (int k = 1; k <= 40; ++k) {
        const double t = 0.05 * static_cast<double>(k);
        imm_.predict(0.05);
        const Eigen::VectorXd z =
            (Eigen::VectorXd(3) << t, 0.0, 0.0).finished();
        imm_.update(z, R_);
    }
    const Eigen::VectorXd x = imm_.getState();
    EXPECT_NEAR(x(0), 2.0, 0.1);   // position after 2 s
    EXPECT_NEAR(x(1), 0.0, 0.05);
    EXPECT_NEAR(x(3), 1.0, 0.2);   // velocity estimate
}

TEST_F(ImmFilterTest, BlendedStateIsConvexCombinationOfModels)
{
    imm_.predict(0.1);
    const Eigen::VectorXd z = (Eigen::VectorXd(3) << 0.1, 0.0, 0.0).finished();
    imm_.update(z, R_);

    const auto mu = imm_.getModelWeights();
    Eigen::VectorXd blended = Eigen::VectorXd::Zero(6);
    for (uint32_t m = 0U; m < 3U; ++m) {
        blended += mu[m] * imm_.getModelState(m).head(6);
    }
    EXPECT_TRUE(imm_.getState().isApprox(blended, 1e-9));
}

TEST_F(ImmFilterTest, CovarianceStaysSymmetric)
{
    for (int k = 1; k <= 10; ++k) {
        imm_.predict(0.05);
        const Eigen::VectorXd z =
            (Eigen::VectorXd(3) << 0.05 * k, 0.0, 0.0).finished();
        imm_.update(z, R_);
    }
    const Eigen::MatrixXd P = imm_.getCovariance();
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-9));
    EXPECT_GT(P(0, 0), 0.0);
}

TEST_F(ImmFilterTest, TurningTargetShiftsWeightTowardCT)
{
    // Coordinated turn: omega = 0.5 rad/s, speed 10 m/s, radius 20 m.
    // Before the setMixedState fix, mixing re-init()ed each model and reset
    // CT's turn rate to zero every cycle, so this maneuver could never move
    // the model weights — this test pins the restored IMM behavior.
    cuas::ImmFilter imm;
    imm.init(make_state(0.0, 0.0, 0.0, 10.0, 0.0, 0.0),
             Eigen::MatrixXd::Identity(6, 6));

    const double omega = 0.5;
    const double radius = 20.0;
    const double dt = 0.05;
    for (int k = 1; k <= 120; ++k) {  // 6 s of sustained turn
        const double t = dt * static_cast<double>(k);
        imm.predict(dt);
        const Eigen::VectorXd z = (Eigen::VectorXd(3)
            << radius * std::sin(omega * t),
               radius * (1.0 - std::cos(omega * t)),
               0.0).finished();
        imm.update(z, R_);
    }

    const auto mu = imm.getModelWeights();
    EXPECT_GT(mu[2], mu[0]) << "CT weight must exceed CV during a turn "
                            << "(cv=" << mu[0] << " ca=" << mu[1]
                            << " ct=" << mu[2] << ")";
    // Blended estimate should still track the turn.
    const Eigen::VectorXd x = imm.getState();
    const double t_end = dt * 120.0;
    EXPECT_NEAR(x(0), radius * std::sin(omega * t_end), 0.5);
    EXPECT_NEAR(x(1), radius * (1.0 - std::cos(omega * t_end)), 0.5);
}

TEST_F(ImmFilterTest, LongStraightRunStaysPsdFiniteAndEveryModeReachable)
{
    // B3 / RC-34, RC-7: 100 s of straight flight with R-consistent noise.
    // Before the block-diagonal mixing fix the blended P went indefinite
    // within 5 s (60/60 seeded runs) and a mode weight hit exactly 0.
    std::mt19937 rng(42U);
    std::normal_distribution<double> noise(0.0, 0.1);   // R_ = 0.01 I
    cuas::ImmFilter imm;
    imm.init(make_state(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
             Eigen::MatrixXd::Identity(6, 6));

    double min_eig = std::numeric_limits<double>::infinity();
    double min_mu  = 1.0;
    for (int k = 1; k <= 2000; ++k) {
        const double t = 0.05 * static_cast<double>(k);
        imm.predict(0.05);
        const Eigen::VectorXd z = (Eigen::VectorXd(3)
            << t + noise(rng), noise(rng), noise(rng)).finished();
        imm.update(z, R_);

        const Eigen::MatrixXd P = imm.getCovariance();
        ASSERT_TRUE(P.allFinite()) << "step " << k;
        ASSERT_TRUE(imm.getState().allFinite()) << "step " << k;
        EXPECT_TRUE(P.isApprox(P.transpose(), 1e-9)) << "step " << k;
        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P, Eigen::EigenvaluesOnly);
        min_eig = std::min(min_eig, es.eigenvalues().minCoeff());
        const auto mu = imm.getModelWeights();
        min_mu = std::min({min_mu, mu[0], mu[1], mu[2]});
    }
    EXPECT_GE(min_eig, -1e-9);
    EXPECT_GE(min_mu, 1e-6);
    EXPECT_NEAR(imm.getState()(3), 1.0, 0.2);
}

TEST_F(ImmFilterTest, TurnAfterLongStraightFlightRecoversCtWeight)
{
    // RC-7: 50 s straight at 10 m/s, then the same coordinated turn as
    // TurningTargetShiftsWeightTowardCT. With the posterior-based weight
    // update mu_CT underflowed during the straight leg and never came back.
    cuas::ImmFilter imm;
    imm.init(make_state(0.0, 0.0, 0.0, 10.0, 0.0, 0.0),
             Eigen::MatrixXd::Identity(6, 6));
    const double dt = 0.05;
    for (int k = 1; k <= 1000; ++k) {
        imm.predict(dt);
        const Eigen::VectorXd z = (Eigen::VectorXd(3)
            << 10.0 * dt * static_cast<double>(k), 0.0, 0.0).finished();
        imm.update(z, R_);
    }
    const auto mu_straight = imm.getModelWeights();
    EXPECT_GE(mu_straight[2], 1e-6);

    const double x0 = 10.0 * dt * 1000.0;
    const double omega = 0.5;
    const double radius = 20.0;
    for (int k = 1; k <= 120; ++k) {
        const double tau = dt * static_cast<double>(k);
        imm.predict(dt);
        const Eigen::VectorXd z = (Eigen::VectorXd(3)
            << x0 + radius * std::sin(omega * tau),
               radius * (1.0 - std::cos(omega * tau)),
               0.0).finished();
        imm.update(z, R_);
    }
    const auto mu = imm.getModelWeights();
    EXPECT_GT(mu[2], 0.5) << "cv=" << mu[0] << " ca=" << mu[1] << " ct=" << mu[2];
}

TEST_F(ImmFilterTest, SetVelocityPropagatesToBlend)
{
    imm_.setVelocity(Eigen::Vector3d(5.0, -2.0, 0.5));
    const Eigen::VectorXd x = imm_.getState();
    EXPECT_NEAR(x(3), 5.0, 1e-9);
    EXPECT_NEAR(x(4), -2.0, 1e-9);
    EXPECT_NEAR(x(5), 0.5, 1e-9);
}

}  // namespace
