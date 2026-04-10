#include "cuas_fusion/prediction/occlusion_predictor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>

#include <map>
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
        double rate = get_parameter("publish_rate_hz").as_double();
        double max_occ = get_parameter("max_occlusion_sec").as_double();
        double gate = get_parameter("mahalanobis_gate").as_double();

        n_steps_ = static_cast<int>(horizon_ / step_dt_);
        predictor_.configure(max_occ, gate);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&OcclusionPredictorNode::trackCallback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/occlusion", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/occlusion", 10);

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        int period_ms = static_cast<int>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&OcclusionPredictorNode::publish, this));

        RCLCPP_INFO(get_logger(), "Occlusion predictor node ready");
    }

private:
    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        double now = clock_->now().seconds();

        for (const auto& t : msg->tracks) {
            if (t.track_state == "OCCLUDED") {
                if (ghost_tracks_.find(t.track_id) == ghost_tracks_.end()) {
                    OcclusionPredictor::GhostTrack ghost;
                    ghost.track_id = t.track_id;
                    ghost.state = Eigen::VectorXd::Zero(6);
                    ghost.state(0) = t.position_x_m;
                    ghost.state(1) = t.position_y_m;
                    ghost.state(2) = t.position_z_m;
                    if (t.velocity_mps > 0.0f) {
                        double bearing = std::atan2(t.position_y_m, t.position_x_m);
                        ghost.state(3) = t.velocity_mps * std::cos(bearing);
                        ghost.state(4) = t.velocity_mps * std::sin(bearing);
                    }
                    ghost.covariance = Eigen::MatrixXd::Zero(6, 6);
                    ghost.covariance.diagonal() << 1.0, 1.0, 1.0, 0.25, 0.25, 0.25;
                    ghost.occlusion_start_time_sec = now;
                    ghost_tracks_[t.track_id] = ghost;
                }
            } else if (t.track_state == "CONFIRMED" || t.track_state == "REACQUIRED") {
                auto it = ghost_tracks_.find(t.track_id);
                if (it != ghost_tracks_.end()) {
                    if (predictor_.reacquire(it->second, t.position_x_m, t.position_y_m, t.position_z_m)) {
                        ghost_tracks_.erase(it);
                    }
                }
            } else if (t.track_state == "LOST") {
                ghost_tracks_.erase(t.track_id);
            }
        }
    }

    void publish()
    {
        double now = clock_->now().seconds();
        std::vector<uint32_t> to_remove;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = step_dt_;
        F(1, 4) = step_dt_;
        F(2, 5) = step_dt_;

        double sa2 = 0.25;
        double dt2 = step_dt_ * step_dt_;
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
        for (int i = 0; i < 3; ++i) {
            Q(i, i)         = 0.25 * dt2 * dt2 * sa2;
            Q(i, i + 3)     = 0.5  * dt2 * step_dt_ * sa2;
            Q(i + 3, i)     = 0.5  * dt2 * step_dt_ * sa2;
            Q(i + 3, i + 3) = dt2 * sa2;
        }

        for (auto& [id, ghost] : ghost_tracks_) {
            auto traj = predictor_.propagateGhost(ghost, now, step_dt_, n_steps_, F, Q);
            if (ghost.expired) {
                to_remove.push_back(id);
                continue;
            }
            if (traj.positions.empty()) continue;

            cuas_msgs::msg::PredictedTrack pred;
            pred.header.stamp = this->now();
            pred.header.frame_id = "base_link";
            pred.track_id = id;

            const auto& last = traj.positions.back();
            pred.pos_x_m = last.x();
            pred.pos_y_m = last.y();
            pred.pos_z_m = last.z();
            pred.vel_x_mps = ghost.state(3);
            pred.vel_y_mps = ghost.state(4);
            pred.vel_z_mps = ghost.state(5);

            for (int r = 0; r < 6; ++r)
                for (int c = 0; c < 6; ++c)
                    pred.covariance[r * 6 + c] = ghost.covariance(r, c);

            pred.bearing_deg = traj.final_bearing_deg;
            pred.elevation_deg = traj.final_elevation_deg;
            pred.model_weight_cv = ghost.model_weights[0];
            pred.model_weight_ca = ghost.model_weights[1];
            pred.model_weight_ct = ghost.model_weights[2];
            pred.track_state = "OCCLUDED";
            pred.prediction_horizon_sec = horizon_;
            pub_pred_->publish(pred);

            cuas_msgs::msg::TrajectoryWaypoints wp;
            wp.header = pred.header;
            wp.track_id = id;
            for (size_t i = 0; i < traj.positions.size(); ++i) {
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

        for (auto id : to_remove) {
            ghost_tracks_.erase(id);
        }
    }

    OcclusionPredictor predictor_;
    double horizon_;
    double step_dt_;
    int n_steps_;

    std::map<uint32_t, OcclusionPredictor::GhostTrack> ghost_tracks_;

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
