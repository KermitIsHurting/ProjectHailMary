// @file cuas_overlay_node.cpp
// @brief ROS 2 node that enhances /camera/annotated with trajectory arcs and track labels.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/ros_image_adapter.hpp"
#include "cuas_fusion/visualization/overlay_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <cstdio>

namespace cuas {

class CuasOverlayNode : public rclcpp::Node
{
public:
    CuasOverlayNode()
    : Node("cuas_overlay_node"),
      overlay_engine_(
          static_cast<float64_t>(CAMERA_FX),
          static_cast<float64_t>(CAMERA_FY),
          static_cast<float64_t>(CAMERA_CX),
          static_cast<float64_t>(CAMERA_CY),
          CAMERA_IMAGE_W, CAMERA_IMAGE_H)
    {
        traj_sub_ = create_subscription<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints", 10,
            std::bind(&CuasOverlayNode::trajectoryCallback, this, std::placeholders::_1));

        track_sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&CuasOverlayNode::trackCallback, this, std::placeholders::_1));

        threat_sub_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
            "/threat/reports", 10,
            std::bind(&CuasOverlayNode::threatCallback, this, std::placeholders::_1));

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/annotated", 5,
            std::bind(&CuasOverlayNode::imageCallback, this, std::placeholders::_1));

        enhanced_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/annotated_enhanced", 5);

        RCLCPP_INFO(get_logger(), "CUAS overlay node ready");
    }

private:
    void trajectoryCallback(
        const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint32_t i = 0U; i < waypoints_.size(); ++i) {
            if (waypoints_[i].track_id == msg->track_id) {
                waypoints_[i] = *msg;
                return;
            }
        }
        (void)waypoints_.push_back(*msg);
    }

    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tracks_.clear();
        const std::size_t count = std::min(
            msg->tracks.size(), static_cast<std::size_t>(TRACK_MAX_TRACKS));
        for (std::size_t i = 0U; i < count; ++i) {
            (void)tracks_.push_back(msg->tracks[i]);
        }

        // Compact out waypoint entries whose track is gone: the cache held
        // 32 lifetime ids, after which trajectory arcs for every new track
        // silently stopped rendering.
        uint32_t kept = 0U;
        for (uint32_t wi = 0U; wi < waypoints_.size(); ++wi) {
            bool live = false;
            for (uint32_t ti = 0U; ti < tracks_.size(); ++ti) {
                if (tracks_[ti].track_id == waypoints_[wi].track_id) {
                    live = true;
                    break;
                }
            }
            if (live) {
                if (kept != wi) {
                    waypoints_[kept] = waypoints_[wi];
                }
                ++kept;
            }
        }
        (void)waypoints_.resize(kept);
    }

    void threatCallback(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        threats_.clear();
        const std::size_t count = std::min(
            msg->reports.size(), static_cast<std::size_t>(TRACK_MAX_TRACKS));
        for (std::size_t i = 0U; i < count; ++i) {
            (void)threats_.push_back(msg->reports[i]);
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        updateFpsEstimate();

        cv::Mat bgr;
        if (!rosImageToBgr(*msg, bgr)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Unsupported image encoding '%s'", msg->encoding.c_str());
            return;
        }
        cv::Mat annotated = bgr.clone();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            overlay_engine_.render(annotated, waypoints_, tracks_, threats_);
        }

        if (!annotated.isContinuous()) {
            return;
        }

        auto out = std::make_unique<sensor_msgs::msg::Image>();
        out->header       = msg->header;
        out->height       = static_cast<uint32_t>(annotated.rows);
        out->width        = static_cast<uint32_t>(annotated.cols);
        out->encoding     = "bgr8";
        out->is_bigendian = 0U;
        out->step         = static_cast<uint32_t>(annotated.cols) * 3U;

        const std::size_t bytes =
            static_cast<std::size_t>(out->step) * static_cast<std::size_t>(out->height);
        out->data.resize(bytes);
        std::memcpy(out->data.data(), annotated.data, bytes);

        enhanced_pub_->publish(std::move(out));
    }

    void updateFpsEstimate()
    {
        const int64_t now_ns = get_clock()->now().nanoseconds();
        const uint32_t write_idx = frame_count_ % FPS_WINDOW;
        frame_times_ns_[write_idx] = now_ns;
        ++frame_count_;

        if (frame_count_ < 2U) {
            return;
        }

        const uint32_t samples =
            (frame_count_ < FPS_WINDOW) ? frame_count_ : FPS_WINDOW;
        const uint32_t newest_idx = write_idx;
        const uint32_t oldest_idx =
            (frame_count_ <= FPS_WINDOW) ? 0U : (frame_count_ % FPS_WINDOW);
        const int64_t delta_ns =
            frame_times_ns_[newest_idx] - frame_times_ns_[oldest_idx];

        if (delta_ns <= 0) {
            return;
        }

        const float64_t seconds = static_cast<float64_t>(delta_ns) * 1.0e-9;
        const float64_t fps =
            static_cast<float64_t>(samples - 1U) / seconds;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "Overlay FPS: %.2f over %u frames (total=%u)",
            fps, samples, frame_count_);
    }

    static constexpr uint32_t FPS_WINDOW = 30U;

    OverlayEngine overlay_engine_;
    FixedVector<cuas_msgs::msg::TrajectoryWaypoints, TRACK_MAX_TRACKS> waypoints_{};
    FixedVector<cuas_msgs::msg::Track,               TRACK_MAX_TRACKS> tracks_{};
    FixedVector<cuas_msgs::msg::ThreatReport,        TRACK_MAX_TRACKS> threats_{};

    rclcpp::Subscription<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr traj_sub_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr          track_sub_;
    rclcpp::Subscription<cuas_msgs::msg::ThreatReportArray>::SharedPtr   threat_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr             image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                enhanced_pub_;
    std::mutex mutex_;

    std::array<int64_t, FPS_WINDOW> frame_times_ns_{};
    uint32_t frame_count_{0U};
};

}  // namespace cuas

// Single sanctioned exception boundary (DEV-001): owned code never
// throws, but rclcpp/rmw, parameter access, and bad_alloc can. Without
// this handler a library throw becomes std::terminate with no fault
// record, invisible to the health monitor. Catch by const ref per
// MISRA C++:2023 18.3.2.
int main(int argc, char ** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<cuas::CuasOverlayNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in CuasOverlayNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in CuasOverlayNode\n");
        exit_code = 1;
    }
    return exit_code;
}
