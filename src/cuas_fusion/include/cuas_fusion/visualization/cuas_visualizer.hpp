#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/fused_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <map>
#include <mutex>
#include <string>

namespace cuas {

class CuasVisualizerNode : public rclcpp::Node
{
public:
    CuasVisualizerNode();

private:
    struct Color { double r, g, b, a; };

    struct CachedDetection {
        cuas_msgs::msg::FusedDetection detection;
        float smooth_u = 0.0f;
        float smooth_v = 0.0f;
        int missed_frames = 0;
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
    std::map<uint32_t, cuas_msgs::msg::PredictedTrack> latest_predictions_;
    std::map<uint32_t, cuas_msgs::msg::TrajectoryWaypoints> latest_trajectories_;
    std::map<uint32_t, std::string> threat_levels_;

    // Temporal smoothing cache (keyed by pixel_u rounded to 50)
    std::map<int, CachedDetection> detection_cache_;

    // Zone dwell timer (keyed by pixel_u rounded to 50)
    std::map<int, std::chrono::steady_clock::time_point> zone_entry_times_;

    // Display toggle parameters
    bool show_prediction_arc_ = true;
    bool show_track_table_ = true;
    bool show_velocity_vector_ = true;
    bool show_ppi_ = true;
    bool show_zone_timer_ = true;
};

} // namespace cuas
