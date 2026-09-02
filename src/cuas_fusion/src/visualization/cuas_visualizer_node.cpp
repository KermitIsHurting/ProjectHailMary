// @file cuas_visualizer_node.cpp
// @brief ROS 2 visualization node rendering tracks, trajectories, PPI, and HUD.
#include "cuas_fusion/common/bearing.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/ros_image_adapter.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/visualization/cuas_visualizer.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <cstdio>

namespace {

inline std::string formatLabeled(const char* prefix,
                                 cuas::float64_t value,
                                 int32_t precision,
                                 const char* suffix = "")
{
    std::ostringstream oss;
    oss << prefix << std::fixed << std::setprecision(precision) << value << suffix;
    return oss.str();
}

} // namespace

namespace cuas {

static constexpr std::array<std::array<float64_t, 3>, 7> kPalette = {{
    {1.0, 0.2, 0.2},
    {0.2, 1.0, 0.2},
    {0.2, 0.2, 1.0},
    {1.0, 1.0, 0.2},
    {1.0, 0.2, 1.0},
    {0.2, 1.0, 1.0},
    {1.0, 0.6, 0.2},
}};

static constexpr std::size_t kCocoNameCount = 80U;
static constexpr std::array<const char*, kCocoNameCount> kCocoNames = {{
    "person", "bicycle", "car", "motorcycle", "airplane",
    "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird",
    "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut",
    "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
}};

CuasVisualizerNode::CuasVisualizerNode()
: Node("cuas_visualizer_node")
{
    declare_parameter("show_prediction_arc", true);
    declare_parameter("show_track_table", true);
    declare_parameter("show_velocity_vector", true);
    declare_parameter("show_ppi", true);
    declare_parameter("show_zone_timer", true);

    show_prediction_arc_  = get_parameter("show_prediction_arc").as_bool();
    show_track_table_     = get_parameter("show_track_table").as_bool();
    show_velocity_vector_ = get_parameter("show_velocity_vector").as_bool();
    show_ppi_             = get_parameter("show_ppi").as_bool();
    show_zone_timer_      = get_parameter("show_zone_timer").as_bool();

    track_sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
        "/tracks", 10,
        std::bind(&CuasVisualizerNode::trackCallback, this, std::placeholders::_1));

    pred_sub_ = create_subscription<cuas_msgs::msg::PredictedTrack>(
        "/predicted_tracks", 10,
        std::bind(&CuasVisualizerNode::predictedTrackCallback, this, std::placeholders::_1));

    traj_sub_ = create_subscription<cuas_msgs::msg::TrajectoryWaypoints>(
        "/trajectory_waypoints", 10,
        std::bind(&CuasVisualizerNode::trajectoryCallback, this, std::placeholders::_1));

    threat_sub_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
        "/threat/reports", 10,
        std::bind(&CuasVisualizerNode::threatCallback, this, std::placeholders::_1));

    track_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/track_markers", 10);
    traj_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/trajectory_markers", 10);
    uncertainty_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/uncertainty_markers", 10);
    label_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/track_labels", 10);

    fused_sub_ = create_subscription<cuas_msgs::msg::FusedDetectionArray>(
        "/fusion/detections", 10,
        std::bind(&CuasVisualizerNode::fusedDetectionCallback, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 1,
        std::bind(&CuasVisualizerNode::imageCallback, this, std::placeholders::_1));

    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/camera/annotated", 1);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CuasVisualizerNode::publishMarkers, this));

    RCLCPP_INFO(get_logger(), "CUAS visualizer node ready (arc=%d table=%d vel=%d ppi=%d zone=%d)",
                show_prediction_arc_, show_track_table_, show_velocity_vector_,
                show_ppi_, show_zone_timer_);
}

void CuasVisualizerNode::trackCallback(
    const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_tracks_ = msg;
}

void CuasVisualizerNode::predictedTrackCallback(
    const cuas_msgs::msg::PredictedTrack::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latest_predictions_.insert_or_assign(msg->track_id, *msg);
}

void CuasVisualizerNode::trajectoryCallback(
    const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    (void)latest_trajectories_.insert_or_assign(msg->track_id, *msg);
}

void CuasVisualizerNode::threatCallback(
    const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_threats_ = msg;
    threat_levels_.clear();
    for (std::size_t i = 0U; i < msg->reports.size(); ++i) {
        const auto& report = msg->reports[i];
        (void)threat_levels_.insert_or_assign(report.track_id, report.threat_level);
    }
}

void CuasVisualizerNode::fusedDetectionCallback(
    const cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_fused_detections_ = msg;
}

void CuasVisualizerNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    cv::Mat base_frame;
    if (!rosImageToBgr(*msg, base_frame)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Unsupported image encoding '%s'", msg->encoding.c_str());
        return;
    }
    cv::Mat annotated = base_frame.clone();

    FixedVector<int32_t, VIZ_MAX_CACHE_ENTRIES> seen_keys{};
    // EMA alpha on pixel position prevents jitter-induced label shake
    constexpr float32_t kAlpha     = 0.2F;
    // Cap the per-frame motion to avoid a single outlier snapping the label
    constexpr float32_t kMaxJumpPx = 150.0F;

    FixedMap<int32_t, int32_t, VIZ_MAX_CACHE_ENTRIES> class_counts{};

    if (latest_fused_detections_) {
        for (std::size_t fi = 0U; fi < latest_fused_detections_->detections.size(); ++fi) {
            const auto& fd = latest_fused_detections_->detections[fi];

            // Quantise pixel column to 50-pixel bins so the cache tolerates small drift
            const int32_t cache_key = static_cast<int32_t>(std::round(fd.pixel_u / 50.0F)) * 50;
            (void)seen_keys.push_back(cache_key);

            const float32_t raw_u = fd.pixel_u;
            const float32_t raw_v = fd.pixel_v;

            float32_t draw_u = raw_u;
            float32_t draw_v = raw_v;
            CachedDetection* cache_entry = detection_cache_.find(cache_key);
            if (cache_entry != nullptr) {
                float32_t du = raw_u - cache_entry->smooth_u;
                float32_t dv = raw_v - cache_entry->smooth_v;
                if (std::abs(du) > kMaxJumpPx) {
                    float32_t du_sign = -1.0F;
                    if (du > 0.0F) {
                        du_sign = 1.0F;
                    }
                    du = du_sign * kMaxJumpPx;
                }
                if (std::abs(dv) > kMaxJumpPx) {
                    float32_t dv_sign = -1.0F;
                    if (dv > 0.0F) {
                        dv_sign = 1.0F;
                    }
                    dv = dv_sign * kMaxJumpPx;
                }
                draw_u = cache_entry->smooth_u + kAlpha * du;
                draw_v = cache_entry->smooth_v + kAlpha * dv;
            }

            CachedDetection fresh;
            fresh.detection     = fd;
            fresh.smooth_u      = draw_u;
            fresh.smooth_v      = draw_v;
            fresh.missed_frames = 0;
            (void)detection_cache_.insert_or_assign(cache_key, fresh);

            std::string threat_level = "UNKNOWN";
            if (latest_threats_) {
                float32_t best_dist = std::numeric_limits<float32_t>::max();
                for (std::size_t ri = 0U; ri < latest_threats_->reports.size(); ++ri) {
                    const auto& report = latest_threats_->reports[ri];
                    const float32_t dx = report.position_x_m - fd.position_x_m;
                    const float32_t dy = report.position_y_m - fd.position_y_m;
                    const float32_t dist = dx * dx + dy * dy;
                    if (dist < best_dist) {
                        best_dist    = dist;
                        threat_level = report.threat_level;
                    }
                }
            }

            cv::Scalar color;
            if (threat_level == "BENIGN") {
                color = cv::Scalar(0, 255, 0);
            } else if (threat_level == "SUSPECT") {
                color = cv::Scalar(0, 255, 255);
            } else if (threat_level == "THREAT") {
                color = cv::Scalar(0, 0, 255);
            } else {
                color = cv::Scalar(255, 255, 255);
            }

            int32_t bw = 0;
            int32_t bh = 0;
            if (fd.bbox_width_px > 0.0F && fd.bbox_height_px > 0.0F) {
                bw = static_cast<int32_t>(fd.bbox_width_px);
                bh = static_cast<int32_t>(fd.bbox_height_px);
            } else {
                // Fallback size if YOLO box is missing — scales inversely with range
                float32_t range = 2.0F;
                if (fd.range_m > 0.1F) {
                    range = fd.range_m;
                }
                bw = static_cast<int32_t>(120.0F * (2.0F / range));
                bh = static_cast<int32_t>(240.0F * (2.0F / range));
            }
            bw = std::max(60, std::min(bw, 350));
            bh = std::max(80, std::min(bh, 500));
            int32_t bx = static_cast<int32_t>(draw_u) - bw / 2;
            // Offset box top by 65% of height so the centroid sits near the torso
            int32_t by = static_cast<int32_t>(draw_v) - static_cast<int32_t>(bh * 0.65F);

            bx = std::max(0, bx);
            by = std::max(0, by);
            const int32_t bx2 = std::min(CAMERA_IMAGE_W - 1, bx + bw);
            const int32_t by2 = std::min(CAMERA_IMAGE_H - 1, by + bh);

            cv::rectangle(annotated, cv::Point(bx, by),
                          cv::Point(bx2, by2), color, 2);

            std::string status;
            if (threat_level == "BENIGN") {
                status = "Safe";
            } else if (threat_level == "SUSPECT") {
                status = "Suspect";
            } else if (threat_level == "THREAT") {
                status = "Warning";
            } else {
                status = "Unknown";
            }

            std::string obj_name = "object";
            const int32_t class_id = parseClassId(fd.class_label);
            if (class_id >= 0 && class_id < static_cast<int32_t>(kCocoNameCount)) {
                obj_name = kCocoNames[static_cast<std::size_t>(class_id)];
            }

            int32_t* cc = class_counts.find(class_id);
            if (cc == nullptr) {
                (void)class_counts.insert_or_assign(class_id, 1);
            } else {
                ++(*cc);
            }
            int32_t obj_num = 1;
            int32_t* cc2 = class_counts.find(class_id);
            if (cc2 != nullptr) {
                obj_num = *cc2;
            }

            std::string label = status + " - " + obj_name;
            if (obj_num > 1) {
                label += " #" + std::to_string(obj_num);
            }
            cv::putText(annotated, label, cv::Point(bx, by - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);

            const cv::Scalar kin_color(0, 255, 255);
            const int32_t text_x = bx + 5;
            const int32_t text_y = by + bh / 2 + 20;

            const std::string r_str = formatLabeled("R: ",
                static_cast<float64_t>(fd.range_m), 1, " m");
            cv::putText(annotated, r_str, cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, kin_color, 1, cv::LINE_AA);

            const std::string a_str = formatLabeled("A: ",
                static_cast<float64_t>(fd.azimuth_deg), 1, " deg");
            cv::putText(annotated, a_str, cv::Point(text_x, text_y + 22),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, kin_color, 1, cv::LINE_AA);

            const std::string v_str = formatLabeled("Vr: ",
                static_cast<float64_t>(fd.velocity_mps), 2, " m/s");
            cv::putText(annotated, v_str, cv::Point(text_x, text_y + 44),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, kin_color, 1, cv::LINE_AA);

            if (show_velocity_vector_ && std::abs(fd.velocity_mps) > 0.05F) {
                const float32_t az_rad = fd.azimuth_deg * static_cast<float32_t>(M_PI) / 180.0F;
                const float32_t end_x  = fd.position_x_m + fd.velocity_mps * std::sin(az_rad) * 3.0F;
                const float32_t end_y  = fd.position_y_m + fd.velocity_mps * std::cos(az_rad) * 3.0F;
                const float32_t end_z  = fd.position_z_m;

                if (end_y > 0.1F) {
                    const float32_t end_u = CAMERA_FX * (end_x / end_y) + CAMERA_CX;
                    const float32_t end_v = CAMERA_FY * (-end_z / end_y) + CAMERA_CY;

                    if (end_u >= 0.0F && end_u < static_cast<float32_t>(CAMERA_IMAGE_W) &&
                        end_v >= 0.0F && end_v < static_cast<float32_t>(CAMERA_IMAGE_H)) {
                        cv::arrowedLine(annotated,
                                        cv::Point(static_cast<int32_t>(draw_u), static_cast<int32_t>(draw_v)),
                                        cv::Point(static_cast<int32_t>(end_u), static_cast<int32_t>(end_v)),
                                        color, 3, cv::LINE_AA, 0, 0.3);
                    }
                }
            }

            if (fd.range_m < 3.0F) {
                if (zone_entry_times_.find(cache_key) == nullptr) {
                    (void)zone_entry_times_.insert_or_assign(
                        cache_key, std::chrono::steady_clock::now());
                }
            } else {
                (void)zone_entry_times_.erase(cache_key);
            }
        }
    }

    // Drop cache entries that missed 3 frames in a row
    for (uint32_t i = 0U; i < detection_cache_.slot_count(); ++i) {
        auto& slot = detection_cache_.slots()[i];
        if (!slot.occupied) {
            continue;
        }
        bool in_seen = false;
        for (uint32_t j = 0U; j < seen_keys.size(); ++j) {
            if (seen_keys[j] == slot.key) {
                in_seen = true;
                break;
            }
        }
        if (!in_seen) {
            ++slot.value.missed_frames;
            if (slot.value.missed_frames > 2) {
                (void)zone_entry_times_.erase(slot.key);
                slot.occupied = false;
            }
        }
    }

    if (show_prediction_arc_ && latest_fused_detections_ &&
        !latest_fused_detections_->detections.empty()) {
        float32_t min_range  = std::numeric_limits<float32_t>::max();
        float32_t closest_az = 0.0F;
        for (std::size_t i = 0U; i < latest_fused_detections_->detections.size(); ++i) {
            const auto& fd = latest_fused_detections_->detections[i];
            if (fd.range_m < min_range) {
                min_range  = fd.range_m;
                closest_az = fd.azimuth_deg;
            }
        }

        for (uint32_t ti = 0U; ti < latest_trajectories_.slot_count(); ++ti) {
            const auto& slot = latest_trajectories_.slots()[ti];
            if (!slot.occupied) {
                continue;
            }
            const auto& traj = slot.value;
            // Cap at 8 waypoints so the arc doesn't overwhelm the frame
            const std::size_t n = std::min(traj.waypoints_x_m.size(), static_cast<std::size_t>(8U));
            if (n < 2U) {
                continue;
            }

            const float32_t traj_az = std::atan2(
                static_cast<float32_t>(traj.waypoints_x_m[0]),
                static_cast<float32_t>(traj.waypoints_y_m[0]))
                * 180.0F / static_cast<float32_t>(M_PI);
            // Only draw the arc when its bearing matches the closest detection
            if (std::abs(traj_az - closest_az) >= 15.0F) {
                continue;
            }

            FixedVector<cv::Point, 16U> arc_pts;
            for (std::size_t i = 0U; i < n; ++i) {
                const float32_t z_cam = static_cast<float32_t>(traj.waypoints_y_m[i]);
                if (z_cam <= 0.1F) {
                    continue;
                }
                const float32_t x_cam = static_cast<float32_t>(traj.waypoints_x_m[i]);
                const float32_t y_cam = -static_cast<float32_t>(traj.waypoints_z_m[i]);
                const float32_t u = CAMERA_FX * (x_cam / z_cam) + CAMERA_CX;
                const float32_t v = CAMERA_FY * (y_cam / z_cam) + CAMERA_CY;

                if (u >= 0.0F && u < static_cast<float32_t>(CAMERA_IMAGE_W) &&
                    v >= 0.0F && v < static_cast<float32_t>(CAMERA_IMAGE_H)) {
                    (void)arc_pts.push_back(cv::Point(static_cast<int32_t>(u), static_cast<int32_t>(v)));

                    if (i < traj.uncertainty_radii_m.size()) {
                        float32_t r_px = static_cast<float32_t>(traj.uncertainty_radii_m[i])
                                         * CAMERA_FX / z_cam;
                        // Cap radius at 30px to prevent viewport fill at close range
                        r_px = std::min(r_px, 30.0F);
                        if (r_px > 2.0F) {
                            cv::ellipse(annotated,
                                        cv::Point(static_cast<int32_t>(u), static_cast<int32_t>(v)),
                                        cv::Size(static_cast<int32_t>(r_px),
                                                 static_cast<int32_t>(r_px * 0.6F)),
                                        0, 0, 360,
                                        cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
                        }
                    }
                }
            }

            if (arc_pts.size() >= 2U) {
                const cv::Mat pts_mat(1, static_cast<int32_t>(arc_pts.size()), CV_32SC2, arc_pts.data());
                cv::polylines(annotated, pts_mat, false, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
                cv::circle(annotated, arc_pts.back(), 8, cv::Scalar(0, 165, 255), cv::FILLED);
            }
            break;
        }
    }

    if (show_zone_timer_) {
        float64_t max_dwell = 0.0;
        const auto now_time = std::chrono::steady_clock::now();
        for (uint32_t i = 0U; i < zone_entry_times_.slot_count(); ++i) {
            const auto& slot = zone_entry_times_.slots()[i];
            if (!slot.occupied) {
                continue;
            }
            const float64_t dwell =
                std::chrono::duration<float64_t>(now_time - slot.value).count();
            if (dwell > max_dwell) {
                max_dwell = dwell;
            }
        }
        // Suppress the readout under 1 second to avoid flicker on transient hits
        if (max_dwell > 1.0) {
            const std::string dwell_str = formatLabeled("Zone: ", max_dwell, 1, "s");
            cv::putText(annotated, dwell_str,
                        cv::Point(annotated.cols - 250, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }
    }

    if (show_track_table_ && latest_fused_detections_ &&
        !latest_fused_detections_->detections.empty()) {
        const int32_t num_rows = std::min(
            static_cast<int32_t>(latest_fused_detections_->detections.size()), 8);
        const int32_t table_w = 320;
        const int32_t table_h = 20 + 18 * num_rows;

        cv::Mat roi = annotated(cv::Rect(0, 0, table_w, table_h));
        cv::Mat overlay(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
        cv::addWeighted(overlay, 0.6, roi, 0.4, 0, roi);

        int32_t row = 0;
        for (std::size_t i = 0U; i < latest_fused_detections_->detections.size(); ++i) {
            if (row >= 8) {
                break;
            }
            const auto& fd = latest_fused_detections_->detections[i];

            std::string tl = "UNK";
            float32_t quality = 0.0F;
            if (latest_threats_) {
                float32_t best_dist = std::numeric_limits<float32_t>::max();
                for (std::size_t ri = 0U; ri < latest_threats_->reports.size(); ++ri) {
                    const auto& r = latest_threats_->reports[ri];
                    const float32_t dx = r.position_x_m - fd.position_x_m;
                    const float32_t dy = r.position_y_m - fd.position_y_m;
                    const float32_t dist = dx * dx + dy * dy;
                    if (dist < best_dist) {
                        best_dist = dist;
                        tl        = r.threat_level;
                        quality   = r.quality_score;
                    }
                }
            }

            std::string tl_short;
            if (tl == "BENIGN") {
                tl_short = "BENIGN";
            } else if (tl == "SUSPECT") {
                tl_short = "SUSPCT";
            } else if (tl == "THREAT") {
                tl_short = "THREAT";
            } else {
                tl_short = "UNK";
            }

            cv::Scalar tl_color;
            if (tl == "BENIGN") {
                tl_color = cv::Scalar(0, 255, 0);
            } else if (tl == "SUSPECT") {
                tl_color = cv::Scalar(0, 255, 255);
            } else if (tl == "THREAT") {
                tl_color = cv::Scalar(0, 0, 255);
            } else {
                tl_color = cv::Scalar(200, 200, 200);
            }

            std::ostringstream row_oss;
            row_oss << "R:" << std::fixed << std::setprecision(1)
                    << static_cast<float64_t>(fd.range_m)
                    << "m Az:" << std::fixed << std::setprecision(0)
                    << static_cast<float64_t>(fd.azimuth_deg) << "d";
            const std::string row_str = row_oss.str();

            const int32_t y = 16 + row * 18;
            cv::putText(annotated, row_str, cv::Point(5, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            cv::putText(annotated, tl_short, cv::Point(175, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, tl_color, 1, cv::LINE_AA);

            const std::string q_str = formatLabeled("Q:",
                static_cast<float64_t>(quality), 2);
            cv::putText(annotated, q_str, cv::Point(265, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            ++row;
        }
    }

    if (show_ppi_) {
        const int32_t sz = 220;
        const int32_t ox = annotated.cols - sz - 10;
        const int32_t oy = annotated.rows - sz - 10;

        // WHY: sector radius constrained by PPI width so the 120 deg arc fits
        const int32_t sector_radius = static_cast<int32_t>(
            static_cast<float32_t>(sz) / (2.0F * std::sin(kPpiFovHalfRad)));
        const int32_t apex_u = ox + sz / 2;
        const int32_t apex_v = oy + sz;
        const float32_t px_per_m =
            static_cast<float32_t>(sector_radius) / kPpiMaxRangeM;

        cv::rectangle(annotated, cv::Point(ox, oy),
                      cv::Point(ox + sz, oy + sz),
                      cv::Scalar(0, 0, 0), cv::FILLED);

        cv::ellipse(annotated, cv::Point(apex_u, apex_v),
                    cv::Size(sector_radius, sector_radius),
                    0.0, 210.0, 330.0,
                    cv::Scalar(30, 30, 30), cv::FILLED);

        const cv::Scalar outline_color(80, 80, 80);
        cv::ellipse(annotated, cv::Point(apex_u, apex_v),
                    cv::Size(sector_radius, sector_radius),
                    0.0, 210.0, 330.0,
                    outline_color, 1);

        const int32_t edge_left_u = apex_u - static_cast<int32_t>(
            static_cast<float32_t>(sector_radius) * std::sin(kPpiFovHalfRad));
        const int32_t edge_top_v = apex_v - static_cast<int32_t>(
            static_cast<float32_t>(sector_radius) * std::cos(kPpiFovHalfRad));
        const int32_t edge_right_u = apex_u + static_cast<int32_t>(
            static_cast<float32_t>(sector_radius) * std::sin(kPpiFovHalfRad));

        cv::line(annotated, cv::Point(apex_u, apex_v),
                 cv::Point(edge_left_u, edge_top_v), outline_color, 1);
        cv::line(annotated, cv::Point(apex_u, apex_v),
                 cv::Point(edge_right_u, edge_top_v), outline_color, 1);

        const cv::Scalar ring_color(60, 60, 60);
        for (int32_t ri = 1; ri <= 2; ++ri) {
            const float32_t ring_range =
                kPpiMaxRangeM * static_cast<float32_t>(ri) / 3.0F;
            const int32_t ring_r = static_cast<int32_t>(ring_range * px_per_m);
            cv::ellipse(annotated, cv::Point(apex_u, apex_v),
                        cv::Size(ring_r, ring_r),
                        0.0, 210.0, 330.0,
                        ring_color, 1);

            const int32_t label_u = apex_u + static_cast<int32_t>(
                static_cast<float32_t>(ring_r) * std::sin(kPpiFovHalfRad));
            const int32_t label_v = apex_v - static_cast<int32_t>(
                static_cast<float32_t>(ring_r) * std::cos(kPpiFovHalfRad));
            const std::string range_str =
                std::to_string(static_cast<int32_t>(ring_range)) + "m";
            cv::putText(annotated, range_str, cv::Point(label_u + 2, label_v),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35,
                        cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
        }

        const cv::Scalar boresight_color(70, 70, 100);
        const int32_t dash_len = 8;
        const int32_t gap_len  = 6;
        int32_t y_cur = apex_v;
        const int32_t y_end = apex_v - sector_radius;
        while (y_cur > y_end) {
            int32_t seg_end = y_cur - dash_len;
            if (seg_end < y_end) {
                seg_end = y_end;
            }
            cv::line(annotated, cv::Point(apex_u, y_cur),
                     cv::Point(apex_u, seg_end), boresight_color, 1);
            y_cur = seg_end - gap_len;
        }

        if (latest_tracks_) {
            for (std::size_t ti = 0U; ti < latest_tracks_->tracks.size(); ++ti) {
                const auto& track = latest_tracks_->tracks[ti];

                const float32_t x_r = track.position_x_m;
                const float32_t y_r = track.position_y_m;
                const float32_t azimuth = std::atan2(x_r, y_r);
                if (std::abs(azimuth) > kPpiFovHalfRad) {
                    continue;
                }

                const float32_t range =
                    std::sqrt(x_r * x_r + y_r * y_r);
                const float32_t r_px = range * px_per_m;
                const int32_t det_u = apex_u +
                    static_cast<int32_t>(r_px * std::sin(azimuth));
                const int32_t det_v = apex_v -
                    static_cast<int32_t>(r_px * std::cos(azimuth));

                if (det_u < ox || det_u >= ox + sz ||
                    det_v < oy || det_v >= oy + sz) {
                    continue;
                }

                const bool is_confirmed =
                    (track.track_state_id == track_state::kConfirmed) ||
                    (track.track_state_id == track_state::kReacquired);

                cv::Scalar dot_color;
                int32_t dot_radius = 0;

                if (is_confirmed) {
                    dot_radius = 5;
                    const std::string* threat =
                        threat_levels_.find(track.track_id);
                    if (threat == nullptr) {
                        dot_color = cv::Scalar(180, 180, 180);
                    } else if (*threat == "BENIGN") {
                        dot_color = cv::Scalar(0, 200, 0);
                    } else if (*threat == "SUSPECT") {
                        dot_color = cv::Scalar(0, 200, 200);
                    } else if (*threat == "THREAT") {
                        dot_color = cv::Scalar(0, 0, 220);
                    } else {
                        dot_color = cv::Scalar(180, 180, 180);
                    }
                } else {
                    dot_radius = 3;
                    dot_color = cv::Scalar(120, 120, 120);
                }

                cv::circle(annotated, cv::Point(det_u, det_v),
                           dot_radius, dot_color, cv::FILLED);

                const std::string id_str =
                    std::to_string(track.track_id);
                cv::putText(annotated, id_str,
                            cv::Point(det_u + dot_radius + 2, det_v + 4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35,
                            dot_color, 1, cv::LINE_AA);
            }
        }

        cv::rectangle(annotated, cv::Point(ox, oy),
                      cv::Point(ox + sz, oy + sz),
                      cv::Scalar(60, 60, 60), 1);

        cv::putText(annotated, "RADAR 120",
                    cv::Point(ox + 5, oy + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40,
                    cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    }

    // Construct the outgoing Image manually so cv_bridge exceptions can't escape
    sensor_msgs::msg::Image out;
    out.header       = msg->header;
    out.height       = static_cast<uint32_t>(annotated.rows);
    out.width        = static_cast<uint32_t>(annotated.cols);
    out.encoding     = "bgr8";
    out.is_bigendian = 0U;
    out.step         = static_cast<uint32_t>(annotated.step);
    const std::size_t data_size = out.step * out.height;
    out.data.assign(annotated.data, annotated.data + data_size);
    annotated_pub_->publish(out);
}

CuasVisualizerNode::Color CuasVisualizerNode::threatColor(
    const std::string& level) const
{
    if (level == "BENIGN")  { return {0.0, 1.0, 0.0, 0.8}; }
    if (level == "UNKNOWN") { return {0.5, 0.5, 0.5, 0.8}; }
    if (level == "SUSPECT") { return {1.0, 1.0, 0.0, 0.8}; }
    if (level == "THREAT")  { return {1.0, 0.0, 0.0, 0.8}; }
    return {0.0, 0.5, 1.0, 0.8};
}

CuasVisualizerNode::Color CuasVisualizerNode::trackPaletteColor(
    uint32_t track_id) const
{
    const auto& c = kPalette[track_id % kPalette.size()];
    return {c[0], c[1], c[2], 0.8};
}

void CuasVisualizerNode::publishMarkers()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_tracks_) {
        return;
    }

    // 100 ms marker lifetime keeps RViz clean when a track disappears
    const auto lifetime = rclcpp::Duration(0, 100000000);

    visualization_msgs::msg::MarkerArray track_markers;
    visualization_msgs::msg::MarkerArray traj_markers;
    visualization_msgs::msg::MarkerArray uncertainty_markers;
    visualization_msgs::msg::MarkerArray label_markers;

    for (std::size_t ti = 0U; ti < latest_tracks_->tracks.size(); ++ti) {
        const auto& track = latest_tracks_->tracks[ti];
        std::string threat = "BENIGN";
        const std::string* threat_ptr = threat_levels_.find(track.track_id);
        if (threat_ptr != nullptr) {
            threat = *threat_ptr;
        }
        const Color tc = threatColor(threat);

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "base_link";
        sphere.header.stamp    = this->now();
        sphere.ns              = "track_positions";
        sphere.id              = static_cast<int32_t>(track.track_id);
        sphere.type            = visualization_msgs::msg::Marker::SPHERE;
        sphere.action          = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = track.position_x_m;
        sphere.pose.position.y = track.position_y_m;
        sphere.pose.position.z = track.position_z_m;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        sphere.color.r = tc.r;
        sphere.color.g = tc.g;
        sphere.color.b = tc.b;
        sphere.color.a = tc.a;
        sphere.lifetime = lifetime;
        track_markers.markers.push_back(sphere);

        visualization_msgs::msg::Marker id_text;
        id_text.header.frame_id = "base_link";
        id_text.header.stamp    = this->now();
        id_text.ns              = "track_id_text";
        id_text.id              = static_cast<int32_t>(track.track_id);
        id_text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        id_text.action          = visualization_msgs::msg::Marker::ADD;
        id_text.pose.position.x = track.position_x_m;
        id_text.pose.position.y = track.position_y_m;
        // Lift the text 0.5 m above the sphere so the label doesn't overlap the marker
        id_text.pose.position.z = static_cast<float64_t>(track.position_z_m) + 0.5;
        id_text.pose.orientation.w = 1.0;
        id_text.scale.z  = 0.25;
        id_text.color.r  = 1.0;
        id_text.color.g  = 1.0;
        id_text.color.b  = 1.0;
        id_text.color.a  = 1.0;
        id_text.lifetime = lifetime;

        float64_t bearing   = 0.0;
        float64_t elevation = 0.0;
        const cuas_msgs::msg::PredictedTrack* pred_entry = latest_predictions_.find(track.track_id);
        if (pred_entry != nullptr) {
            bearing   = pred_entry->bearing_deg;
            elevation = pred_entry->elevation_deg;
        } else {
            const float64_t px = track.position_x_m;
            const float64_t py = track.position_y_m;
            const float64_t pz = track.position_z_m;
            bearing   = static_cast<float64_t>(bearingDegBoresightZero(
                static_cast<float32_t>(px), static_cast<float32_t>(py)));
            elevation = std::atan2(pz, std::sqrt(px * px + py * py)) * 180.0 / M_PI;
        }

        std::ostringstream oss;
        oss << "ID:" << track.track_id << " " << track.track_state << "\n"
            << std::fixed << std::setprecision(1)
            << bearing << "deg " << elevation << "deg";
        id_text.text = oss.str();
        label_markers.markers.push_back(id_text);

        const cuas_msgs::msg::TrajectoryWaypoints* traj_entry =
            latest_trajectories_.find(track.track_id);
        if (traj_entry != nullptr) {
            const auto& traj = *traj_entry;
            const Color pc = trackPaletteColor(track.track_id);
            const std::size_t n = traj.waypoints_x_m.size();

            if (n > 0U) {
                visualization_msgs::msg::Marker line;
                line.header.frame_id = "base_link";
                line.header.stamp    = this->now();
                line.ns              = "trajectories";
                line.id              = static_cast<int32_t>(track.track_id);
                line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
                line.action          = visualization_msgs::msg::Marker::ADD;
                line.pose.orientation.w = 1.0;
                line.scale.x = 0.08;
                line.color.r = pc.r;
                line.color.g = pc.g;
                line.color.b = pc.b;
                line.color.a = pc.a;
                line.lifetime = lifetime;

                for (std::size_t i = 0U; i < n; ++i) {
                    geometry_msgs::msg::Point pt;
                    pt.x = traj.waypoints_x_m[i];
                    pt.y = traj.waypoints_y_m[i];
                    pt.z = traj.waypoints_z_m[i];
                    line.points.push_back(pt);
                }
                traj_markers.markers.push_back(line);

                for (std::size_t i = 0U; i < n; ++i) {
                    float64_t radius = 0.1;
                    if (i < traj.uncertainty_radii_m.size()) {
                        radius = traj.uncertainty_radii_m[i];
                    }
                    // Clamp radius to [0.05, 0.5] m so RViz stays usable
                    radius = std::min(radius, 0.5);
                    radius = std::max(radius, 0.05);

                    uint32_t denom = 1U;
                    if (n > 1U) {
                        denom = n - 1U;
                    }
                    float64_t alpha = 0.3 - (static_cast<float64_t>(i)
                        / static_cast<float64_t>(denom)) * 0.25;
                    if (alpha < 0.05) {
                        alpha = 0.05;
                    }

                    visualization_msgs::msg::Marker usphere;
                    usphere.header.frame_id = "base_link";
                    usphere.header.stamp    = this->now();
                    usphere.ns              = "uncertainty_" + std::to_string(track.track_id);
                    usphere.id              = static_cast<int32_t>(i);
                    usphere.type            = visualization_msgs::msg::Marker::SPHERE;
                    usphere.action          = visualization_msgs::msg::Marker::ADD;
                    usphere.pose.position.x = traj.waypoints_x_m[i];
                    usphere.pose.position.y = traj.waypoints_y_m[i];
                    usphere.pose.position.z = traj.waypoints_z_m[i];
                    usphere.pose.orientation.w = 1.0;
                    usphere.scale.x = radius * 2.0;
                    usphere.scale.y = radius * 2.0;
                    usphere.scale.z = radius * 2.0;
                    usphere.color.r = pc.r;
                    usphere.color.g = pc.g;
                    usphere.color.b = pc.b;
                    usphere.color.a = alpha;
                    usphere.lifetime = lifetime;
                    uncertainty_markers.markers.push_back(usphere);
                }
            }
        }
    }

    track_marker_pub_->publish(track_markers);
    traj_marker_pub_->publish(traj_markers);
    uncertainty_marker_pub_->publish(uncertainty_markers);
    label_pub_->publish(label_markers);
}

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
        auto node = std::make_shared<cuas::CuasVisualizerNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in CuasVisualizerNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in CuasVisualizerNode\n");
        exit_code = 1;
    }
    return exit_code;
}
