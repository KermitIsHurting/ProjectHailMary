// @file overlay_engine.hpp
// @brief Pinhole reprojection and OpenCV overlay drawing for camera-annotated frames.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/types.hpp"

#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>
#include <cuas_msgs/msg/threat_report.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <string>

namespace cuas {

class OverlayEngine {
public:
    OverlayEngine(float64_t fx, float64_t fy,
                  float64_t cx, float64_t cy,
                  int32_t image_width, int32_t image_height);

    void render(
        cv::Mat& image,
        const FixedVector<cuas_msgs::msg::TrajectoryWaypoints, TRACK_MAX_TRACKS>& waypoints,
        const FixedVector<cuas_msgs::msg::Track, TRACK_MAX_TRACKS>& tracks,
        const FixedVector<cuas_msgs::msg::ThreatReport, TRACK_MAX_TRACKS>& threat_reports);

private:
    void draw_trajectory_arc(
        cv::Mat& image,
        const cuas_msgs::msg::TrajectoryWaypoints& wp,
        const cv::Scalar& color);

    void draw_scaled_label(
        cv::Mat& image,
        const cv::Point2i& origin,
        const std::string& text,
        float32_t font_scale,
        int32_t thickness,
        const cv::Scalar& bg_color,
        const cv::Scalar& text_color,
        bool use_background,
        float32_t label_alpha);

    cv::Point2i project_to_image(
        float64_t x_radar,
        float64_t y_radar,
        float64_t z_radar) const;

    cv::Scalar threat_color(ThreatLevel level) const;

    bool is_in_bounds(const cv::Point2i& pt) const;

    float64_t fx_;
    float64_t fy_;
    float64_t cx_;
    float64_t cy_;
    int32_t   image_width_;
    int32_t   image_height_;
};

}  // namespace cuas
