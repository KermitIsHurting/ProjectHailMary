#include "cuas_fusion/tracking/imm_tracker.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>

#include <map>
#include <cmath>
#include <limits>

namespace cuas {

class IMMTrackerNode : public rclcpp::Node
{
public:
    IMMTrackerNode()
    : Node("imm_tracker_node")
    , next_track_id_(1)
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10,
            std::bind(&IMMTrackerNode::radarCallback, this, std::placeholders::_1));

        pub_ = create_publisher<cuas_msgs::msg::TrackArray>("/tracks", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&IMMTrackerNode::publishTracks, this));

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        last_predict_time_ = clock_->now().seconds();

        RCLCPP_INFO(get_logger(), "IMM tracker node ready");
    }

private:
    static constexpr double kAssociationGate = 0.8;

    void radarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        double now = clock_->now().seconds();

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            double px = *iter_x;
            double py = *iter_y;
            double pz = *iter_z;

            uint32_t best_id = 0;
            double best_dist = kAssociationGate;

            for (auto& [id, tracker] : active_tracks_) {
                Eigen::VectorXd pos = tracker.getPosition();
                double dx = pos(0) - px;
                double dy = pos(1) - py;
                double dz = pos(2) - pz;
                double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (d < best_dist) {
                    best_dist = d;
                    best_id = id;
                }
            }

            if (best_id != 0) {
                active_tracks_.at(best_id).update(px, py, pz, now);
            } else {
                uint32_t new_id = next_track_id_++;
                active_tracks_.emplace(new_id, IMMTracker(new_id, px, py, pz, now));
            }
        }
    }

    void publishTracks()
    {
        double now = clock_->now().seconds();
        double dt = now - last_predict_time_;
        last_predict_time_ = now;
        if (dt <= 0.0) dt = 0.05;

        std::vector<uint32_t> to_remove;

        for (auto& [id, tracker] : active_tracks_) {
            tracker.predict(dt);

            double elapsed = now - tracker.lastUpdateTime();
            std::string state = tracker.getState();

            if ((state == "CONFIRMED" || state == "REACQUIRED") && elapsed > 0.5) {
                // transition to OCCLUDED handled externally via state string
            }
            if (elapsed > 5.0) {
                to_remove.push_back(id);
            }
        }

        for (auto id : to_remove) {
            active_tracks_.erase(id);
        }

        cuas_msgs::msg::TrackArray out;
        out.header.stamp = this->now();
        out.header.frame_id = "base_link";

        for (const auto& [id, tracker] : active_tracks_) {
            cuas_msgs::msg::Track t;
            t.track_id = tracker.getTrackId();
            Eigen::VectorXd pos = tracker.getPosition();
            Eigen::VectorXd vel = tracker.getVelocity();
            t.position_x_m = static_cast<float>(pos(0));
            t.position_y_m = static_cast<float>(pos(1));
            t.position_z_m = static_cast<float>(pos(2));
            t.velocity_mps = static_cast<float>(vel.norm());
            t.doppler_mps  = 0.0f;
            t.class_label  = "unknown";
            t.confidence   = 1.0f;
            t.track_state  = tracker.getState();
            t.timestamp_ns = static_cast<int64_t>(tracker.lastUpdateTime() * 1e9);
            out.tracks.push_back(t);
        }

        pub_->publish(out);
    }

    std::map<uint32_t, IMMTracker> active_tracks_;
    uint32_t next_track_id_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<cuas_msgs::msg::TrackArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Clock> clock_;
    double last_predict_time_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::IMMTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
