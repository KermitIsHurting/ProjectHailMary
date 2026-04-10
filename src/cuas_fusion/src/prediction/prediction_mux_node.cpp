#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>

#include <map>
#include <vector>

namespace cuas {

class PredictionMuxNode : public rclcpp::Node
{
public:
    PredictionMuxNode()
    : Node("prediction_mux_node")
    {
        sub_kin_pred_ = create_subscription<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/kinematic", 10,
            std::bind(&PredictionMuxNode::kinPredCallback, this, std::placeholders::_1));

        sub_occ_pred_ = create_subscription<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/occlusion", 10,
            std::bind(&PredictionMuxNode::occPredCallback, this, std::placeholders::_1));

        sub_kin_traj_ = create_subscription<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/kinematic", 10,
            std::bind(&PredictionMuxNode::kinTrajCallback, this, std::placeholders::_1));

        sub_occ_traj_ = create_subscription<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/occlusion", 10,
            std::bind(&PredictionMuxNode::occTrajCallback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PredictionMuxNode::mergeTick, this));

        RCLCPP_INFO(get_logger(), "Prediction mux node ready");
    }

private:
    void kinPredCallback(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr& msg)
    {
        kinematic_pred_[msg->track_id] = *msg;
    }

    void occPredCallback(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr& msg)
    {
        occlusion_pred_[msg->track_id] = *msg;
    }

    void kinTrajCallback(const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
    {
        kinematic_traj_[msg->track_id] = *msg;
    }

    void occTrajCallback(const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
    {
        occlusion_traj_[msg->track_id] = *msg;
    }

    void mergeTick()
    {
        std::map<uint32_t, bool> seen;

        for (const auto& [id, _] : kinematic_pred_) seen[id] = true;
        for (const auto& [id, _] : occlusion_pred_) seen[id] = true;

        for (const auto& [id, _] : seen) {
            auto kit = kinematic_pred_.find(id);
            auto oit = occlusion_pred_.find(id);

            const cuas_msgs::msg::PredictedTrack* chosen = nullptr;

            if (kit != kinematic_pred_.end() && oit != occlusion_pred_.end()) {
                if (oit->second.track_state == "OCCLUDED") {
                    chosen = &oit->second;
                } else {
                    chosen = &kit->second;
                }
            } else if (oit != occlusion_pred_.end()) {
                chosen = &oit->second;
            } else if (kit != kinematic_pred_.end()) {
                chosen = &kit->second;
            }

            if (chosen) {
                pub_pred_->publish(*chosen);
            }

            // Same logic for trajectory
            auto kt = kinematic_traj_.find(id);
            auto ot = occlusion_traj_.find(id);

            const cuas_msgs::msg::TrajectoryWaypoints* chosen_t = nullptr;

            if (kt != kinematic_traj_.end() && ot != occlusion_traj_.end()) {
                if (oit != occlusion_pred_.end() && oit->second.track_state == "OCCLUDED") {
                    chosen_t = &ot->second;
                } else {
                    chosen_t = &kt->second;
                }
            } else if (ot != occlusion_traj_.end()) {
                chosen_t = &ot->second;
            } else if (kt != kinematic_traj_.end()) {
                chosen_t = &kt->second;
            }

            if (chosen_t) {
                pub_traj_->publish(*chosen_t);
            }
        }
    }

    std::map<uint32_t, cuas_msgs::msg::PredictedTrack> kinematic_pred_;
    std::map<uint32_t, cuas_msgs::msg::PredictedTrack> occlusion_pred_;
    std::map<uint32_t, cuas_msgs::msg::TrajectoryWaypoints> kinematic_traj_;
    std::map<uint32_t, cuas_msgs::msg::TrajectoryWaypoints> occlusion_traj_;

    rclcpp::Subscription<cuas_msgs::msg::PredictedTrack>::SharedPtr sub_kin_pred_;
    rclcpp::Subscription<cuas_msgs::msg::PredictedTrack>::SharedPtr sub_occ_pred_;
    rclcpp::Subscription<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr sub_kin_traj_;
    rclcpp::Subscription<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr sub_occ_traj_;

    rclcpp::Publisher<cuas_msgs::msg::PredictedTrack>::SharedPtr pub_pred_;
    rclcpp::Publisher<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr pub_traj_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::PredictionMuxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
