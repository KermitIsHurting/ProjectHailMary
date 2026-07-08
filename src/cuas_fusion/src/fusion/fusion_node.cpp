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

#include <array>
#include <charconv>
#include <cmath>
#include <ctime>
#include <mutex>
#include <string>
#include <system_error>
#include <vector>
#include <cstdio>

namespace cuas {

namespace {

// YOLO boxes older than this are not fused against live radar tracks (A1.7)
constexpr int64_t kYoloMaxAgeNs = 500'000'000LL;

// Local monotonic clock for the staleness gate: the camera pipeline stamps
// CLOCK_MONOTONIC while the tracker stamps ROS system time, so message
// stamps cannot be compared across the two streams.
inline int64_t monotonicNowNs()
{
    struct timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL
         + static_cast<int64_t>(ts.tv_nsec);
}

} // namespace

class FusionNode : public rclcpp::Node
{
public:
    FusionNode()
    : Node("fusion_node")
    {
        // Radar→camera SE(3): p_cam = R(q)·p_radar + t (see
        // config/extrinsics.yaml; defaults = nominal mount + tape-measure
        // lever arm re-expressed in camera axes).
        declare_parameter<std::vector<float64_t>>("extrinsics.rotation_wxyz",
            {0.70710678, 0.70710678, 0.0, 0.0});
        declare_parameter<std::vector<float64_t>>("extrinsics.translation_m",
            {-0.0075, 0.079, 0.017});

        const std::vector<float64_t> q =
            get_parameter("extrinsics.rotation_wxyz").as_double_array();
        const std::vector<float64_t> t =
            get_parameter("extrinsics.translation_m").as_double_array();
        if (q.size() != 4U || t.size() != 3U) {
            RCLCPP_FATAL(get_logger(),
                "extrinsics.rotation_wxyz needs 4 values (got %zu), "
                "extrinsics.translation_m needs 3 (got %zu)",
                q.size(), t.size());
            rclcpp::shutdown();
            return;
        }

        ExtrinsicTransform ext;
        ext.q_w   = static_cast<float32_t>(q[0]);
        ext.q_x   = static_cast<float32_t>(q[1]);
        ext.q_y   = static_cast<float32_t>(q[2]);
        ext.q_z   = static_cast<float32_t>(q[3]);
        ext.t_x_m = static_cast<float32_t>(t[0]);
        ext.t_y_m = static_cast<float32_t>(t[1]);
        ext.t_z_m = static_cast<float32_t>(t[2]);

        if (!engine_.init(ext)) {
            RCLCPP_FATAL(get_logger(),
                "FusionEngine init failed: non-finite or degenerate extrinsics");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Extrinsics q_wxyz=[%.6f %.6f %.6f %.6f] t_cam=[%.4f %.4f %.4f] m",
            q[0], q[1], q[2], q[3], t[0], t[1], t[2]);

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "radar_frame";
        tf_msg.child_frame_id  = "camera_frame";
        // TF carries the CAMERA pose in the RADAR frame — the inverse of the
        // measurement mapping: R_tf = Rᵀ (conjugate q), t_tf = -Rᵀ·t.
        std::array<float32_t, 9> rot{};
        (void)extrinsicRotationMatrix(ext, rot);  // validated by init() above
        const float64_t qn = std::sqrt((q[0] * q[0]) + (q[1] * q[1]) +
                                       (q[2] * q[2]) + (q[3] * q[3]));
        tf_msg.transform.rotation.w = q[0] / qn;
        tf_msg.transform.rotation.x = -q[1] / qn;
        tf_msg.transform.rotation.y = -q[2] / qn;
        tf_msg.transform.rotation.z = -q[3] / qn;
        tf_msg.transform.translation.x = -static_cast<float64_t>(
            (rot[0] * ext.t_x_m) + (rot[3] * ext.t_y_m) + (rot[6] * ext.t_z_m));
        tf_msg.transform.translation.y = -static_cast<float64_t>(
            (rot[1] * ext.t_x_m) + (rot[4] * ext.t_y_m) + (rot[7] * ext.t_z_m));
        tf_msg.transform.translation.z = -static_cast<float64_t>(
            (rot[2] * ext.t_x_m) + (rot[5] * ext.t_y_m) + (rot[8] * ext.t_z_m));
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
            if (monotonicNowNs() - latest_yolo_rx_ns_ > kYoloMaxAgeNs) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "YOLO detections older than %d ms — skipping label fusion",
                    static_cast<int32_t>(kYoloMaxAgeNs / 1'000'000LL));
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
        out.detections.reserve(fused.size());

        for (std::size_t i = 0U; i < fused.size(); ++i) {
            const FusedDetection& fd = fused[i];
            cuas_msgs::msg::FusedDetection det;
            det.position_x_m = fd.position_x_m;
            det.position_y_m = fd.position_y_m;
            det.position_z_m = fd.position_z_m;
            det.velocity_mps = fd.velocity_mps;
            det.class_label  = classIdToLabel(fd.class_id);
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
        latest_yolo_rx_ns_ = monotonicNowNs();

        // An empty detection array is valid information — the frame really
        // contains no targets — so it clears the cache rather than leaving
        // the previous boxes to be fused forever (A1.7).
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
    int64_t latest_yolo_rx_ns_ = 0;
};

} // namespace cuas

// Single sanctioned exception boundary (DEV-001): owned code never
// throws, but rclcpp/rmw, parameter access, and bad_alloc can. Without
// this handler a library throw becomes std::terminate with no fault
// record, invisible to the health monitor. Catch by const ref per
// MISRA C++:2023 18.3.2.
int main(int argc, char** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<cuas::FusionNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in FusionNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in FusionNode\n");
        exit_code = 1;
    }
    return exit_code;
}
