
// fusion_node.cpp

#include "cuas_fusion/fusion/fusion_engine.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cuas_msgs/msg/fused_detection.hpp>
#include <cuas_msgs/msg/fused_detection_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <vector>
#include <mutex>

namespace cuas {

class FusionNode : public rclcpp::Node
{
public:
    FusionNode()
    : Node("fusion_node")
    {
        declare_parameter<double>("extrinsic.x_offset_m", -0.0075);
        declare_parameter<double>("extrinsic.y_offset_m",  0.017);
        declare_parameter<double>("extrinsic.z_offset_m", -0.079);

        ExtrinsicTransform ext;
        ext.x_m = static_cast<float>(get_parameter("extrinsic.x_offset_m").as_double());
        ext.y_m = static_cast<float>(get_parameter("extrinsic.y_offset_m").as_double());
        ext.z_m = static_cast<float>(get_parameter("extrinsic.z_offset_m").as_double());

        if (!engine_.init(ext)) {
            RCLCPP_FATAL(get_logger(), "FusionEngine init failed");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Extrinsic offsets: x=%.4f y=%.4f z=%.4f",
                     ext.x_m, ext.y_m, ext.z_m);

        // TF2 static: radar_frame → camera_frame (identity rotation)
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "radar_frame";
        tf_msg.child_frame_id  = "camera_frame";
        tf_msg.transform.translation.x = static_cast<double>(ext.x_m);
        tf_msg.transform.translation.y = static_cast<double>(ext.y_m);
        tf_msg.transform.translation.z = static_cast<double>(ext.z_m);
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);

        pub_ = create_publisher<cuas_msgs::msg::FusedDetectionArray>(
            "/fusion/detections", 5);

        radar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 1,
            std::bind(&FusionNode::radarCallback, this, std::placeholders::_1));

        yolo_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
            "/inference/detections", 1,
            std::bind(&FusionNode::yoloCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Fusion node ready");
    }

private:
    void radarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        std::vector<RadarDetection> radar_pts;
        radar_pts.reserve(msg->width);

        int64_t ts_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL
                      + static_cast<int64_t>(msg->header.stamp.nanosec);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> it_v(*msg, "velocity");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_v) {
            RadarDetection rd;
            rd.x = *it_x;
            rd.y = *it_y;
            rd.z = *it_z;
            rd.velocity = *it_v;
            rd.timestamp_ns = ts_ns;
            radar_pts.push_back(rd);
        }

        std::vector<BoundingBox> yolo_boxes;
        {
            std::lock_guard<std::mutex> lock(yolo_mutex_);
            if (latest_yolo_boxes_.empty()) {
                return;
            }
            int64_t delta = (ts_ns > latest_yolo_ts_)
                          ? (ts_ns - latest_yolo_ts_)
                          : (latest_yolo_ts_ - ts_ns);
            if (delta > MAX_TIMESTAMP_DELTA_NS) {
                return;
            }
            yolo_boxes = latest_yolo_boxes_;
        }

        std::vector<FusedDetection> fused;
        if (!engine_.projectAndAssociate(radar_pts, yolo_boxes, fused)) {
            return;
        }

        if (fused.empty()) {
            return;
        }

        cuas_msgs::msg::FusedDetectionArray out;
        out.header = msg->header;
        out.header.frame_id = "radar_frame";

        for (const auto& fd : fused) {
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

        int64_t ts_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL
                      + static_cast<int64_t>(msg->header.stamp.nanosec);
        latest_yolo_ts_ = ts_ns;

        if (msg->detections.empty()) {
            return;  // Keep last known boxes
        }
        latest_yolo_boxes_.clear();

        for (const auto& det : msg->detections) {
            BoundingBox bb;
            // Detection2D bbox is center+size; convert to top-left+w/h
            bb.x = static_cast<float>(det.bbox.center.position.x - det.bbox.size_x * 0.5);
            bb.y = static_cast<float>(det.bbox.center.position.y - det.bbox.size_y * 0.5);
            bb.w = static_cast<float>(det.bbox.size_x);
            bb.h = static_cast<float>(det.bbox.size_y);

            if (!det.results.empty()) {
                bb.confidence = static_cast<float>(det.results[0].hypothesis.score);
                bb.class_id   = std::stoi(det.results[0].hypothesis.class_id);
            } else {
                bb.confidence = 0.0f;
                bb.class_id   = -1;
            }
            bb.timestamp_ns = ts_ns;

            latest_yolo_boxes_.push_back(bb);
        }
    }

    FusionEngine engine_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<cuas_msgs::msg::FusedDetectionArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;

    std::mutex yolo_mutex_;
    std::vector<BoundingBox> latest_yolo_boxes_;
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
