// @file cuas_visualizer.hpp
// @brief ROS 2 visualization node that renders tracks, trajectories, and PPI.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/fused_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace cuas {

static constexpr uint32_t VIZ_MAX_CACHE_ENTRIES = 64U;

class CuasVisualizerNode : public rclcpp::Node
{
public:
    CuasVisualizerNode();

private:
    struct Color {
        float64_t r = 0.0;
        float64_t g = 0.0;
        float64_t b = 0.0;
        float64_t a = 0.0;
    };

    struct CachedDetection {
        cuas_msgs::msg::FusedDetection detection;
        float32_t smooth_u     = 0.0F;
        float32_t smooth_v     = 0.0F;
        int32_t   missed_frames = 0;
    };

    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg);
    void predictedTrackCallback(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr& msg);
    void trajectoryCallback(const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg);
    void threatCallback(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg);
    void fusedDetectionCallback(const cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr& msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void publishMarkers();

    Color threatColor(const std::string& level) const;
    Color trackPaletteColor(uint32_t track_id) const;

    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr track_sub_;
    rclcpp::Subscription<cuas_msgs::msg::PredictedTrack>::SharedPtr pred_sub_;
    rclcpp::Subscription<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr traj_sub_;
    rclcpp::Subscription<cuas_msgs::msg::ThreatReportArray>::SharedPtr threat_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uncertainty_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr label_pub_;

    rclcpp::Subscription<cuas_msgs::msg::FusedDetectionArray>::SharedPtr fused_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr latest_fused_detections_;
    cuas_msgs::msg::ThreatReportArray::ConstSharedPtr latest_threats_;
    cuas_msgs::msg::TrackArray::ConstSharedPtr latest_tracks_;
    // ~15 Hz annotated output (RC-26).
    static constexpr int64_t kVizMinAnnotatePeriodNs = 66'000'000LL;
    int64_t last_annotated_stamp_ns_ = 0;
    FixedMap<uint32_t, cuas_msgs::msg::PredictedTrack,      TRACK_MAX_TRACKS> latest_predictions_{};
    FixedMap<uint32_t, cuas_msgs::msg::TrajectoryWaypoints, TRACK_MAX_TRACKS> latest_trajectories_{};
    FixedMap<uint32_t, std::string,                         TRACK_MAX_TRACKS> threat_levels_{};

    FixedMap<int32_t, CachedDetection,                         VIZ_MAX_CACHE_ENTRIES> detection_cache_{};
    FixedMap<int32_t, std::chrono::steady_clock::time_point,   VIZ_MAX_CACHE_ENTRIES> zone_entry_times_{};

    bool show_prediction_arc_  = true;
    bool show_track_table_     = true;
    bool show_velocity_vector_ = true;
    bool show_ppi_             = true;
    bool show_zone_timer_      = true;
};

} // namespace cuas
