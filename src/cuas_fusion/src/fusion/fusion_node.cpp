// @file fusion_node.cpp
// @brief ROS 2 node wrapping FusionEngine with track and YOLO subscriptions.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/fusion/detection_set_buffer.hpp"
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
#include <mutex>
#include <string>
#include <system_error>
#include <vector>
#include <cstdio>

namespace cuas {

namespace {

inline int64_t stampToNs(const builtin_interfaces::msg::Time& t)
{
    return (static_cast<int64_t>(t.sec) * 1'000'000'000LL) +
           static_cast<int64_t>(t.nanosec);
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
        // Per-measurement temporal alignment (P2.2): pick the camera
        // detection set whose stamp is nearest the track stamp (both
        // CLOCK_MONOTONIC per ICD §2), then extrapolate each track to
        // that instant with its own velocity before projecting. This
        // replaces the latest-box-within-500-ms-of-arrival heuristic,
        // whose association error grew linearly with target speed.
        const int64_t t_track_ns = stampToNs(msg->header.stamp);

        // No tracks: nothing to label, and no reason to look up a camera
        // set (R10-10). The empty array still goes out so the classifier
        // learns there are no fused labels now (RC-5a).
        if (msg->tracks.empty()) {
            publishFused({}, t_track_ns);
            return;
        }

        DetectionSetBuffer::BoxSet yolo_boxes;
        int64_t t_cam_ns = 0;
        {
            std::lock_guard<std::mutex> lock(yolo_mutex_);
            if (!yolo_buffer_.selectNearest(t_track_ns, MAX_TIMESTAMP_DELTA_NS,
                                            yolo_boxes, t_cam_ns)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "no camera detections within %d ms of track stamp — "
                    "skipping label fusion",
                    static_cast<int32_t>(MAX_TIMESTAMP_DELTA_NS / 1'000'000LL));
                publishFused({}, t_track_ns);
                return;
            }
        }

        const float32_t dt_s =
            static_cast<float32_t>(t_cam_ns - t_track_ns) * 1.0e-9F;

        FixedVector<RadarDetection, TRACK_MAX_TRACKS> radar_pts;
        for (std::size_t i = 0U; i < msg->tracks.size(); ++i) {
            const auto& track = msg->tracks[i];
            RadarDetection rd;
            // Track state extrapolated to the camera instant, all three
            // axes: holding z put a 3 m/s climb 169 px off its box (RC-20).
            rd.x = track.position_x_m + (track.vx_mps * dt_s);
            rd.y = track.position_y_m + (track.vy_mps * dt_s);
            rd.z = track.position_z_m + (track.vz_mps * dt_s);
            rd.velocity = track.velocity_mps;
            rd.timestamp_ns = t_cam_ns;
            if (!radar_pts.push_back(rd)) {
                break;
            }
        }

        FixedVector<FusedDetection, TRACK_MAX_TRACKS> fused;
        if (!engine_.projectAndAssociate(radar_pts, yolo_boxes, fused)) {
            return;
        }

        // Output state is valid at the camera instant it was aligned to.
        // Published even when empty (RC-5a): the classifier held its last
        // non-empty set forever and re-applied stale labels.
        publishFused(fused, t_cam_ns);
    }

    void publishFused(const FixedVector<FusedDetection, TRACK_MAX_TRACKS>& fused,
                      int64_t stamp_ns)
    {
        cuas_msgs::msg::FusedDetectionArray out;
        out.header.stamp.sec     = static_cast<int32_t>(stamp_ns / 1'000'000'000LL);
        out.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000LL);
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
            // P3.1: engine output is radar-anchored; the camera bit is set
            // iff a YOLO label joined this detection.
            det.source_mask = (fd.class_id >= 0)
                ? static_cast<uint8_t>(track_source::kRadar | track_source::kCamera)
                : track_source::kRadar;
            out.detections.push_back(det);
        }

        pub_->publish(out);
    }

    void yoloCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg)
    {
        const int64_t ts_ns = stampToNs(msg->header.stamp);

        // An empty detection array is valid information — the frame really
        // contains no targets — so it is buffered as an empty set: a track
        // aligned to that instant fuses against nothing (A1.7 semantics).
        DetectionSetBuffer::BoxSet boxes;
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

            if (!boxes.push_back(bb)) {
                break;
            }
        }

        std::lock_guard<std::mutex> lock(yolo_mutex_);
        yolo_buffer_.addSet(boxes, ts_ns);
    }

    FusionEngine engine_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<cuas_msgs::msg::FusedDetectionArray>::SharedPtr pub_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr track_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_sub_;

    std::mutex yolo_mutex_;
    DetectionSetBuffer yolo_buffer_;
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
