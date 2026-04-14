// @file kinematic_predictor_node.cpp
// @brief ROS 2 node that projects tracks forward and publishes predictions.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/estimation/imm_filter.hpp"
#include "cuas_fusion/prediction/kinematic_predictor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>

namespace cuas {

struct CovarianceCache {
    Eigen::Matrix<float64_t, 6, 6> P =
        Eigen::Matrix<float64_t, 6, 6>::Identity();
    std::array<float64_t, 3> weights = {0.33, 0.33, 0.34};
};

class KinematicPredictorNode : public rclcpp::Node
{
public:
    KinematicPredictorNode()
    : Node("kinematic_predictor_node")
    {
        declare_parameter("prediction_horizon_sec", 5.0);
        declare_parameter("prediction_step_dt", 0.1);
        declare_parameter("publish_rate_hz", 20.0);

        horizon_ = get_parameter("prediction_horizon_sec").as_double();
        step_dt_ = get_parameter("prediction_step_dt").as_double();
        const float64_t rate = get_parameter("publish_rate_hz").as_double();

        n_steps_ = static_cast<int32_t>(horizon_ / step_dt_);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&KinematicPredictorNode::trackCallback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/kinematic", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/kinematic", 10);

        const int32_t period_ms = static_cast<int32_t>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&KinematicPredictorNode::publish, this));

        RCLCPP_INFO(get_logger(), "Kinematic predictor node ready (horizon=%.1fs, dt=%.2fs)",
                     horizon_, step_dt_);
    }

private:
    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        latest_tracks_ = *msg;

        for (const auto& t : msg->tracks) {
            if (cov_cache_.find(t.track_id) == nullptr) {
                CovarianceCache fresh;
                fresh.P = Eigen::Matrix<float64_t, 6, 6>::Zero();
                fresh.P.diagonal() << 1.0, 1.0, 1.0, 0.25, 0.25, 0.25;
                fresh.weights = {0.33, 0.33, 0.34};
                (void)cov_cache_.insert_or_assign(t.track_id, fresh);
            }
        }
    }

    void publish()
    {
        for (const auto& t : latest_tracks_.tracks) {
            if (t.track_state != "CONFIRMED" && t.track_state != "REACQUIRED") {
                continue;
            }

            Eigen::VectorXd state(6);
            state << t.position_x_m, t.position_y_m, t.position_z_m,
                     0.0, 0.0, 0.0;

            if (t.velocity_mps > 0.0F) {
                const float64_t bearing = std::atan2(
                    static_cast<float64_t>(t.position_y_m),
                    static_cast<float64_t>(t.position_x_m));
                state(3) = static_cast<float64_t>(t.velocity_mps) * std::cos(bearing);
                state(4) = static_cast<float64_t>(t.velocity_mps) * std::sin(bearing);
            }

            CovarianceCache* cache = cov_cache_.find(t.track_id);
            if (cache == nullptr) {
                continue;
            }
            Eigen::MatrixXd P = cache->P;
            const std::array<float64_t, 3> weights = cache->weights;

            Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
            F(0, 3) = step_dt_;
            F(1, 4) = step_dt_;
            F(2, 5) = step_dt_;

            const float64_t sa2 = 0.25;
            const float64_t dt2 = step_dt_ * step_dt_;
            Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
            for (int32_t i = 0; i < 3; ++i) {
                Q(i, i)         = 0.25 * dt2 * dt2 * sa2;
                Q(i, i + 3)     = 0.5  * dt2 * step_dt_ * sa2;
                Q(i + 3, i)     = 0.5  * dt2 * step_dt_ * sa2;
                Q(i + 3, i + 3) = dt2 * sa2;
            }

            auto traj = predictor_.propagateForward(state, P, weights, F, Q, step_dt_, n_steps_);

            if (!traj.positions.empty()) {
                const Eigen::MatrixXd P_prop = F * P * F.transpose() + Q;
                cache->P = P_prop;
            }

            cuas_msgs::msg::PredictedTrack pred;
            pred.header.stamp = this->now();
            pred.header.frame_id = "base_link";
            pred.track_id = t.track_id;

            if (!traj.positions.empty()) {
                const auto& last = traj.positions.back();
                pred.pos_x_m = last.x();
                pred.pos_y_m = last.y();
                pred.pos_z_m = last.z();
            }
            pred.vel_x_mps = state(3);
            pred.vel_y_mps = state(4);
            pred.vel_z_mps = state(5);

            const Eigen::MatrixXd Pcov = cache->P;
            for (int32_t r = 0; r < 6; ++r) {
                for (int32_t c = 0; c < 6; ++c) {
                    pred.covariance[static_cast<uint32_t>(r * 6 + c)] = Pcov(r, c);
                }
            }

            pred.bearing_deg            = traj.final_bearing_deg;
            pred.elevation_deg          = traj.final_elevation_deg;
            pred.model_weight_cv        = weights[0];
            pred.model_weight_ca        = weights[1];
            pred.model_weight_ct        = weights[2];
            pred.track_state            = t.track_state;
            pred.prediction_horizon_sec = horizon_;
            pub_pred_->publish(pred);

            cuas_msgs::msg::TrajectoryWaypoints wp;
            wp.header   = pred.header;
            wp.track_id = t.track_id;
            for (std::size_t i = 0; i < traj.positions.size(); ++i) {
                wp.waypoints_x_m.push_back(traj.positions[i].x());
                wp.waypoints_y_m.push_back(traj.positions[i].y());
                wp.waypoints_z_m.push_back(traj.positions[i].z());
                wp.timestamps_sec.push_back(traj.timestamps_sec[i]);
                wp.uncertainty_radii_m.push_back(traj.uncertainty_radii_m[i]);
                wp.bearing_deg.push_back(traj.bearing_deg[i]);
                wp.elevation_deg.push_back(traj.elevation_deg[i]);
            }
            pub_traj_->publish(wp);
        }
    }

    KinematicPredictor predictor_;
    float64_t horizon_ = 0.0;
    float64_t step_dt_ = 0.0;
    int32_t   n_steps_ = 0;

    cuas_msgs::msg::TrackArray latest_tracks_;
    FixedMap<uint32_t, CovarianceCache, TRACK_MAX_TRACKS> cov_cache_{};

    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
    rclcpp::Publisher<cuas_msgs::msg::PredictedTrack>::SharedPtr pub_pred_;
    rclcpp::Publisher<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr pub_traj_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::KinematicPredictorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
