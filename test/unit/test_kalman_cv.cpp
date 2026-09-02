// test_kalman_cv.cpp
// Unit tests for the constant-velocity Kalman filter: predict/update
// correctness, covariance growth during coast, and measurement convergence.

#include "cuas_fusion/estimation/kalman_cv.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace {

Eigen::VectorXd make_state(double px, double py, double pz,
                           double vx, double vy, double vz)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    x << px, py, pz, vx, vy, vz;
    return x;
}

class KalmanCvTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        filter_.init(make_state(1.0, 2.0, 3.0, 0.5, -0.5, 0.0),
                     Eigen::MatrixXd::Identity(6, 6));
    }
    cuas::KalmanCV filter_;
};

TEST_F(KalmanCvTest, InitStoresState)
{
    const Eigen::VectorXd x = filter_.getState();
    ASSERT_EQ(x.size(), 6);
    EXPECT_DOUBLE_EQ(x(0), 1.0);
    EXPECT_DOUBLE_EQ(x(3), 0.5);
    EXPECT_TRUE(filter_.getCovariance().isApprox(Eigen::MatrixXd::Identity(6, 6)));
}

TEST_F(KalmanCvTest, PredictIntegratesVelocity)
{
    filter_.predict(2.0);
    const Eigen::VectorXd x = filter_.getState();
    EXPECT_NEAR(x(0), 1.0 + 0.5 * 2.0, 1e-12);   // px + vx*dt
    EXPECT_NEAR(x(1), 2.0 - 0.5 * 2.0, 1e-12);
    EXPECT_NEAR(x(2), 3.0, 1e-12);
    EXPECT_NEAR(x(3), 0.5, 1e-12);               // velocity unchanged
}

TEST_F(KalmanCvTest, CovarianceGrowsDuringCoast)
{
    const double p0 = filter_.getCovariance()(0, 0);
    for (int i = 0; i < 5; ++i) {
        filter_.predict(0.1);
    }
    EXPECT_GT(filter_.getCovariance()(0, 0), p0);
}

TEST_F(KalmanCvTest, UpdateConvergesToRepeatedMeasurement)
{
    const Eigen::VectorXd z = (Eigen::VectorXd(3) << 10.0, 20.0, 30.0).finished();
    const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 0.01;
    for (int i = 0; i < 30; ++i) {
        filter_.predict(0.05);
        filter_.update(z, R);
    }
    const Eigen::VectorXd x = filter_.getState();
    EXPECT_NEAR(x(0), 10.0, 0.1);
    EXPECT_NEAR(x(1), 20.0, 0.1);
    EXPECT_NEAR(x(2), 30.0, 0.1);
    // Position covariance must have contracted from 1.0 toward R's scale.
    EXPECT_LT(filter_.getCovariance()(0, 0), 0.1);
}

TEST_F(KalmanCvTest, TransitionMatrixStructure)
{
    const Eigen::MatrixXd F = filter_.getF(0.25);
    EXPECT_DOUBLE_EQ(F(0, 3), 0.25);
    EXPECT_DOUBLE_EQ(F(1, 4), 0.25);
    EXPECT_DOUBLE_EQ(F(2, 5), 0.25);
    EXPECT_TRUE(F.diagonal().isApprox(Eigen::VectorXd::Ones(6)));
}

TEST_F(KalmanCvTest, ProcessNoiseSymmetricPositiveSemiDefinite)
{
    const Eigen::MatrixXd Q = filter_.getQ(0.1);
    EXPECT_TRUE(Q.isApprox(Q.transpose()));
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q);
    EXPECT_GE(es.eigenvalues().minCoeff(), -1e-15);
}

TEST_F(KalmanCvTest, IndefiniteInnovationCovarianceSkipsUpdateAndFloorsLikelihood)
{
    // B3: a P that has lost positive definiteness makes S non-SPD; the
    // Cholesky guard keeps the prediction instead of applying a gain built
    // from S.inverse(), and the likelihood takes its floor.
    cuas::KalmanCV kf;
    const Eigen::VectorXd x0 = make_state(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    kf.init(x0, -Eigen::MatrixXd::Identity(6, 6));
    const Eigen::Vector3d z(5.0, 5.0, 5.0);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    kf.update(z, R);
    EXPECT_TRUE(kf.getState().isApprox(x0, 1e-12));
    EXPECT_DOUBLE_EQ(kf.likelihood(z, R), 1e-12);
}

TEST_F(KalmanCvTest, LikelihoodHigherNearPredictedState)
{
    const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    const Eigen::VectorXd near = (Eigen::VectorXd(3) << 1.0, 2.0, 3.0).finished();
    const Eigen::VectorXd far  = (Eigen::VectorXd(3) << 50.0, 50.0, 50.0).finished();
    EXPECT_GT(filter_.likelihood(near, R), filter_.likelihood(far, R));
    EXPECT_GE(filter_.likelihood(far, R), 1e-12);  // floored, never zero
}

}  // namespace
