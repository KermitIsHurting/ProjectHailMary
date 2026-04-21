// @file fusion_node.cpp
// @brief ROS 2 node wrapping FusionEngine with track and YOLO subscriptions.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/fusion/fusion_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cuas_msgs/msg/fused_detection.hpp>
#include <cuas_msgs/msg/fused_detection_array.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <charconv>
#include <mutex>
#include <string>
#include <system_error>

namespace cuas {

namespace {

inline int32_t parseClassId(const std::string& s)
{
    if (s.empty()) {
        return -1;
    }
    int32_t value = 0;
    const auto result = std::from_chars(s.data(), s.data() + s.size(), value);
    if (result.ec != std::errc{}) {
        return -1;
    }
    return value;
}

} // namespace

class FusionNode : public rclcpp::Node
{
public:
    FusionNode()
    : Node("fusion_node")
    {
        declare_parameter<float64_t>("extrinsic.x_offset_m", -0.0075);
        declare_parameter<float64_t>("extrinsic.y_offset_m",  0.017);
        declare_parameter<float64_t>("extrinsic.z_offset_m", -0.079);

        ExtrinsicTransform ext;
        ext.x_m = static_cast<float32_t>(get_parameter("extrinsic.x_offset_m").as_double());
        ext.y_m = static_cast<float32_t>(get_parameter("extrinsic.y_offset_m").as_double());
        ext.z_m = static_cast<float32_t>(get_parameter("extrinsic.z_offset_m").as_double());

        if (!engine_.init(ext)) {
            RCLCPP_FATAL(get_logger(), "FusionEngine init failed");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Extrinsic offsets: x=%.4f y=%.4f z=%.4f",
                     static_cast<float64_t>(ext.x_m),
                     static_cast<float64_t>(ext.y_m),
                     static_cast<float64_t>(ext.z_m));

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "radar_frame";
        tf_msg.child_frame_id  = "camera_frame";
        tf_msg.transform.translation.x = static_cast<float64_t>(ext.x_m);
        tf_msg.transform.translation.y = static_cast<float64_t>(ext.y_m);
        tf_msg.transform.translation.z = static_cast<float64_t>(ext.z_m);
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);

        pub_ = create_publisher<cuas_msgs::msg::FusedDetectionArray>(
            "/fusion/detections", 5);

        track_sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 1,
            std::bind(&FusionNode::trackCallback, this, std::placeholders::_1));

        yolo_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
            "/inference/detections", 1,
            std::bind(&FusionNode::yoloCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Fusion node ready");
    }

private:
    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        FixedVector<RadarDetection, TRACK_MAX_TRACKS> radar_pts;

        for (std::size_t i = 0U; i < msg->tracks.size(); ++i) {
            const auto& track = msg->tracks[i];
            RadarDetection rd;
            rd.x = track.position_x_m;
            rd.y = track.position_y_m;
            rd.z = track.position_z_m;
            rd.velocity = track.velocity_mps;
            rd.timestamp_ns = track.timestamp_ns;
            if (!radar_pts.push_back(rd)) {
                break;
            }
        }

        FixedVector<BoundingBox, 128U> yolo_boxes;
        {
            std::lock_guard<std::mutex> lock(yolo_mutex_);
            if (latest_yolo_boxes_.empty()) {
                return;
            }
            yolo_boxes = latest_yolo_boxes_;
        }

        FixedVector<FusedDetection, TRACK_MAX_TRACKS> fused;
        if (!engine_.projectAndAssociate(radar_pts, yolo_boxes, fused)) {
            return;
        }

        if (fused.empty()) {
            return;
        }

        cuas_msgs::msg::FusedDetectionArray out;
        out.header = msg->header;
        out.header.frame_id = "radar_frame";

        for (std::size_t i = 0U; i < fused.size(); ++i) {
            const FusedDetection& fd = fused[i];
            cuas_msgs::msg::FusedDetection det;
            det.position_x_m = fd.position_x_m;
            det.position_y_m = fd.position_y_m;
            det.position_z_m = fd.position_z_m;
            det.velocity_mps = fd.velocity_mps;
            det.class_label  = fd.class_label;
            det.confidence   = fd.confidence;
            det.pixel_u      = fd.pixel_u;
            det.pixel_v      = fd.pixel_v;
            det.timestamp_ns = fd.timestamp_ns;
            det.range_m      = fd.range_m;
            det.azimuth_deg  = fd.azimuth_deg;
            det.bbox_width_px  = fd.bbox_width_px;
            det.bbox_height_px = fd.bbox_height_px;
            out.detections.push_back(det);
        }

        pub_->publish(out);
    }

    void yoloCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(yolo_mutex_);

        const int64_t ts_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL
                      + static_cast<int64_t>(msg->header.stamp.nanosec);
        latest_yolo_ts_ = ts_ns;

        if (msg->detections.empty()) {
            return;
        }
        latest_yolo_boxes_.clear();

        for (std::size_t i = 0U; i < msg->detections.size(); ++i) {
            const auto& det = msg->detections[i];
            BoundingBox bb;
            bb.x = static_cast<float32_t>(det.bbox.center.position.x - det.bbox.size_x * 0.5);
            bb.y = static_cast<float32_t>(det.bbox.center.position.y - det.bbox.size_y * 0.5);
            bb.w = static_cast<float32_t>(det.bbox.size_x);
            bb.h = static_cast<float32_t>(det.bbox.size_y);

            if (!det.results.empty()) {
                bb.confidence = static_cast<float32_t>(det.results[0].hypothesis.score);
                bb.class_id   = parseClassId(det.results[0].hypothesis.class_id);
            } else {
                bb.confidence = 0.0F;
                bb.class_id   = -1;
            }
            bb.timestamp_ns = ts_ns;

            if (!latest_yolo_boxes_.push_back(bb)) {
                break;
            }
        }
    }

    FusionEngine engine_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<cuas_msgs::msg::FusedDetectionArray>::SharedPtr pub_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr track_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;

    std::mutex yolo_mutex_;
    FixedVector<BoundingBox, 128U> latest_yolo_boxes_;
    int64_t latest_yolo_ts_ = 0;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
