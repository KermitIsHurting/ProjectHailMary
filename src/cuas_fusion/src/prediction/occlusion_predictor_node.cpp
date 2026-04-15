// @file occlusion_predictor_node.cpp
// @brief ROS 2 node that maintains ghost tracks and publishes their forecasts.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/prediction/occlusion_predictor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>

#include <cmath>

namespace cuas {

class OcclusionPredictorNode : public rclcpp::Node
{
public:
    OcclusionPredictorNode()
    : Node("occlusion_predictor_node")
    {
        declare_parameter("prediction_horizon_sec", 5.0);
        declare_parameter("prediction_step_dt", 0.1);
        declare_parameter("publish_rate_hz", 20.0);
        declare_parameter("max_occlusion_sec", 4.0);
        declare_parameter("mahalanobis_gate", 3.0);

        horizon_ = get_parameter("prediction_horizon_sec").as_double();
        step_dt_ = get_parameter("prediction_step_dt").as_double();
        const float64_t rate    = get_parameter("publish_rate_hz").as_double();
        const float64_t max_occ = get_parameter("max_occlusion_sec").as_double();
        const float64_t gate    = get_parameter("mahalanobis_gate").as_double();

        n_steps_ = static_cast<int32_t>(horizon_ / step_dt_);
        predictor_.configure(max_occ, gate);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&OcclusionPredictorNode::trackCallback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/occlusion", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/occlusion", 10);

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        const int32_t period_ms = static_cast<int32_t>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&OcclusionPredictorNode::publish, this));

        RCLCPP_INFO(get_logger(), "Occlusion predictor node ready");
    }

private:
    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        const float64_t now = clock_->now().seconds();

        for (const auto& t : msg->tracks) {
            (void)horizon_cache_.insert_or_assign(t.track_id, t.prediction_horizon_s);
            if (t.track_state == "OCCLUDED") {
                if (ghost_tracks_.find(t.track_id) == nullptr) {
                    OcclusionPredictor::GhostTrack ghost;
                    ghost.track_id = t.track_id;
                    ghost.state    = Eigen::VectorXd::Zero(6);
                    ghost.state(0) = t.position_x_m;
                    ghost.state(1) = t.position_y_m;
                    ghost.state(2) = t.position_z_m;
                    if (t.velocity_mps > 0.0F) {
                        const float64_t bearing = std::atan2(
                            static_cast<float64_t>(t.position_y_m),
                            static_cast<float64_t>(t.position_x_m));
                        ghost.state(3) = static_cast<float64_t>(t.velocity_mps) * std::cos(bearing);
                        ghost.state(4) = static_cast<float64_t>(t.velocity_mps) * std::sin(bearing);
                    }
                    ghost.covariance = Eigen::MatrixXd::Zero(6, 6);
                    ghost.covariance.diagonal() << 1.0, 1.0, 1.0, 0.25, 0.25, 0.25;
                    ghost.occlusion_start_time_sec = now;
                    (void)ghost_tracks_.insert_or_assign(t.track_id, ghost);
                }
            } else if (t.track_state == "CONFIRMED" || t.track_state == "REACQUIRED") {
                OcclusionPredictor::GhostTrack* ghost = ghost_tracks_.find(t.track_id);
                if (ghost != nullptr) {
                    if (predictor_.reacquire(*ghost,
                                             t.position_x_m,
                                             t.position_y_m,
                                             t.position_z_m)) {
                        (void)ghost_tracks_.erase(t.track_id);
                    }
                }
            } else if (t.track_state == "LOST") {
                (void)ghost_tracks_.erase(t.track_id);
            } else {
            }
        }
    }

    void publish()
    {
        const float64_t now = clock_->now().seconds();

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

        for (uint32_t s = 0U; s < ghost_tracks_.slot_count(); ++s) {
            auto& slot = ghost_tracks_.slots()[s];
            if (!slot.occupied) {
                continue;
            }
            OcclusionPredictor::GhostTrack& ghost = slot.value;

            auto traj = predictor_.propagateGhost(ghost, now, step_dt_, n_steps_, F, Q);
            if (ghost.expired) {
                slot.occupied = false;
                continue;
            }
            if (traj.positions.empty()) {
                continue;
            }

            cuas_msgs::msg::PredictedTrack pred;
            pred.header.stamp    = this->now();
            pred.header.frame_id = "base_link";
            pred.track_id        = slot.key;

            const auto& last = traj.positions.back();
            pred.pos_x_m = last.x();
            pred.pos_y_m = last.y();
            pred.pos_z_m = last.z();
            pred.vel_x_mps = ghost.state(3);
            pred.vel_y_mps = ghost.state(4);
            pred.vel_z_mps = ghost.state(5);

            for (int32_t r = 0; r < 6; ++r) {
                for (int32_t c = 0; c < 6; ++c) {
                    pred.covariance[static_cast<uint32_t>(r * 6 + c)] = ghost.covariance(r, c);
                }
            }

            pred.bearing_deg            = traj.final_bearing_deg;
            pred.elevation_deg          = traj.final_elevation_deg;
            pred.model_weight_cv        = ghost.model_weights[0];
            pred.model_weight_ca        = ghost.model_weights[1];
            pred.model_weight_ct        = ghost.model_weights[2];
            pred.track_state            = "OCCLUDED";
            // WHY: prediction_horizon_s is stamped onto Track by the tracker
            // node which owns the single /threat/reports join — direct read
            // here, no policy ownership in the predictor.
            const float32_t* h_ptr = horizon_cache_.find(slot.key);
            pred.prediction_horizon_sec = static_cast<float64_t>(
                (h_ptr != nullptr) ? *h_ptr : 5.0F);
            pub_pred_->publish(pred);

            cuas_msgs::msg::TrajectoryWaypoints wp;
            wp.header   = pred.header;
            wp.track_id = slot.key;
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

    OcclusionPredictor predictor_;
    float64_t horizon_ = 0.0;
    float64_t step_dt_ = 0.0;
    int32_t   n_steps_ = 0;

    FixedMap<uint32_t, OcclusionPredictor::GhostTrack, TRACK_MAX_TRACKS> ghost_tracks_{};
    FixedMap<uint32_t, float32_t, TRACK_MAX_TRACKS> horizon_cache_{};

    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
    rclcpp::Publisher<cuas_msgs::msg::PredictedTrack>::SharedPtr pub_pred_;
    rclcpp::Publisher<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr pub_traj_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Clock> clock_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::OcclusionPredictorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
