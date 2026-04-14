// @file track_manager_node.cpp
// @brief ROS 2 node wrapping TrackManager for fused-detection input.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/fused_detection_array.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/track.hpp>

namespace cuas {

class TrackerNode : public rclcpp::Node
{
public:
    TrackerNode()
    : Node("tracker_node")
    {
        if (!manager_.init()) {
            RCLCPP_FATAL(get_logger(), "TrackManager init failed");
            rclcpp::shutdown();
            return;
        }

        pub_ = create_publisher<cuas_msgs::msg::TrackArray>("/tracks/confirmed", 5);

        sub_ = create_subscription<cuas_msgs::msg::FusedDetectionArray>(
            "/fusion/detections", 5,
            std::bind(&TrackerNode::fusionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Tracker node ready");
    }

private:
    void fusionCallback(const cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr& msg)
    {
        FixedVector<FusedDetection, TRACK_MAX_TRACKS> detections;
        for (const auto& d : msg->detections) {
            if (detections.size() >= detections.capacity()) {
                break;
            }
            FusedDetection fd;
            fd.position_x_m = d.position_x_m;
            fd.position_y_m = d.position_y_m;
            fd.position_z_m = d.position_z_m;
            fd.velocity_mps = d.velocity_mps;
            fd.class_label  = d.class_label;
            fd.confidence   = d.confidence;
            fd.timestamp_ns = d.timestamp_ns;
            (void)detections.push_back(fd);
        }

        FixedVector<Track, TRACK_MAX_TRACKS> confirmed;
        if (!manager_.update(detections.data(), detections.size(), confirmed)) {
            RCLCPP_WARN(get_logger(), "TrackManager update failed");
            return;
        }

        cuas_msgs::msg::TrackArray out;
        out.header = msg->header;

        for (uint32_t i = 0U; i < confirmed.size(); ++i) {
            const Track& t = confirmed[i];
            cuas_msgs::msg::Track tm;
            tm.track_id      = t.track_id_;
            tm.position_x_m  = t.position_x_m_;
            tm.position_y_m  = t.position_y_m_;
            tm.position_z_m  = t.position_z_m_;
            tm.velocity_mps  = t.velocity_mps_;
            tm.doppler_mps   = t.doppler_mps_;
            tm.class_label   = t.class_label_;
            tm.confidence    = t.confidence_;
            tm.track_state   = trackStateToString(t.state_);
            tm.timestamp_ns  = t.timestamp_ns_;
            out.tracks.push_back(tm);
        }

        pub_->publish(out);
    }

    TrackManager manager_;
    rclcpp::Publisher<cuas_msgs::msg::TrackArray>::SharedPtr pub_;
    rclcpp::Subscription<cuas_msgs::msg::FusedDetectionArray>::SharedPtr sub_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::TrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
