#include "cuas_fusion/visualization/cuas_visualizer.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace cuas {

static constexpr std::array<std::array<double, 3>, 7> kPalette = {{
    {1.0, 0.2, 0.2},
    {0.2, 1.0, 0.2},
    {0.2, 0.2, 1.0},
    {1.0, 1.0, 0.2},
    {1.0, 0.2, 1.0},
    {0.2, 1.0, 1.0},
    {1.0, 0.6, 0.2},
}};

CuasVisualizerNode::CuasVisualizerNode()
: Node("cuas_visualizer_node")
{
    // Load display toggle parameters
    declare_parameter("show_prediction_arc", true);
    declare_parameter("show_track_table", true);
    declare_parameter("show_velocity_vector", true);
    declare_parameter("show_ppi", true);
    declare_parameter("show_zone_timer", true);

    show_prediction_arc_ = get_parameter("show_prediction_arc").as_bool();
    show_track_table_ = get_parameter("show_track_table").as_bool();
    show_velocity_vector_ = get_parameter("show_velocity_vector").as_bool();
    show_ppi_ = get_parameter("show_ppi").as_bool();
    show_zone_timer_ = get_parameter("show_zone_timer").as_bool();

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
        "/camera/image_raw", 1,
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
    latest_predictions_[msg->track_id] = *msg;
}

void CuasVisualizerNode::trajectoryCallback(
    const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_trajectories_[msg->track_id] = *msg;
}

void CuasVisualizerNode::threatCallback(
    const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_threats_ = msg;
    threat_levels_.clear();
    for (const auto& report : msg->reports) {
        threat_levels_[report.track_id] = report.threat_level;
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

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        return;
    }
    cv::Mat annotated = cv_ptr->image.clone();

    std::set<int> seen_keys;
    constexpr float kAlpha = 0.35f;

    static const std::vector<std::string> kCocoNames = {
        "person", "bicycle", "car", "motorcycle", "airplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird",
        "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
        "carrot", "hot dog", "pizza", "donut", "cake", "chair",
        "couch", "potted plant", "bed", "dining table", "toilet",
        "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator",
        "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"
    };
    std::map<int, int> class_counts;

    // Draw fused detections with threat-colored bboxes and kinematics
    if (latest_fused_detections_) {
        for (const auto& fd : latest_fused_detections_->detections) {
            int cache_key = static_cast<int>(std::round(fd.pixel_u / 50.0f)) * 50;
            seen_keys.insert(cache_key);

            // Step A: Try Kalman track projection for smoother position
            float raw_u = fd.pixel_u, raw_v = fd.pixel_v;
            if (latest_tracks_) {
                float best_az_diff = 15.0f;
                for (const auto& track : latest_tracks_->tracks) {
                    float track_az = std::atan2(track.position_x_m, track.position_y_m)
                                     * 180.0f / static_cast<float>(M_PI);
                    float az_diff = std::abs(track_az - fd.azimuth_deg);
                    if (az_diff < best_az_diff && track.position_y_m > 0.1f) {
                        best_az_diff = az_diff;
                        raw_u = CAMERA_FX * (track.position_x_m / track.position_y_m) + CAMERA_CX;
                        raw_v = CAMERA_FY * (-track.position_z_m / track.position_y_m) + CAMERA_CY;
                    }
                }
            }

            // EMA position smoothing
            float draw_u, draw_v;
            auto cache_it = detection_cache_.find(cache_key);
            if (cache_it != detection_cache_.end()) {
                draw_u = kAlpha * raw_u + (1.0f - kAlpha) * cache_it->second.smooth_u;
                draw_v = kAlpha * raw_v + (1.0f - kAlpha) * cache_it->second.smooth_v;
            } else {
                draw_u = raw_u;
                draw_v = raw_v;
            }
            detection_cache_[cache_key] = {fd, draw_u, draw_v, 0};

            // Match to closest ThreatReport by position proximity
            std::string threat_level = "UNKNOWN";
            if (latest_threats_) {
                float best_dist = std::numeric_limits<float>::max();
                for (const auto& report : latest_threats_->reports) {
                    float dx = report.position_x_m - fd.position_x_m;
                    float dy = report.position_y_m - fd.position_y_m;
                    float dist = dx * dx + dy * dy;
                    if (dist < best_dist) {
                        best_dist = dist;
                        threat_level = report.threat_level;
                    }
                }
            }

            // Bbox color by threat level
            cv::Scalar color;
            if (threat_level == "BENIGN")       color = cv::Scalar(0, 255, 0);
            else if (threat_level == "SUSPECT") color = cv::Scalar(0, 255, 255);
            else if (threat_level == "THREAT")  color = cv::Scalar(0, 0, 255);
            else                                color = cv::Scalar(255, 255, 255);

            // Step B: Bbox size from YOLO dimensions or range-scaled fallback
            int bw, bh;
            if (fd.bbox_width_px > 0 && fd.bbox_height_px > 0) {
                bw = static_cast<int>(fd.bbox_width_px);
                bh = static_cast<int>(fd.bbox_height_px);
            } else {
                float range = fd.range_m > 0.1f ? fd.range_m : 2.0f;
                bw = static_cast<int>(120.0f * (2.0f / range));
                bh = static_cast<int>(240.0f * (2.0f / range));
            }
            bw = std::max(60, std::min(bw, 350));
            bh = std::max(80, std::min(bh, 500));
            int bx = static_cast<int>(draw_u) - bw / 2;
            int by = static_cast<int>(draw_v) - static_cast<int>(bh * 0.65f);

            bx = std::max(0, bx);
            by = std::max(0, by);
            int bx2 = std::min(CAMERA_IMAGE_W - 1, bx + bw);
            int by2 = std::min(CAMERA_IMAGE_H - 1, by + bh);

            cv::rectangle(annotated, cv::Point(bx, by),
                          cv::Point(bx2, by2), color, 2);

            // Threat label + object type above bbox
            std::string status;
            if (threat_level == "BENIGN")       status = "Safe";
            else if (threat_level == "SUSPECT") status = "Suspect";
            else if (threat_level == "THREAT")  status = "Warning";
            else                                status = "Unknown";

            std::string obj_name = "object";
            int class_id = -1;
            try { class_id = std::stoi(fd.class_label); } catch (...) {}
            if (class_id >= 0 && class_id < static_cast<int>(kCocoNames.size())) {
                obj_name = kCocoNames[class_id];
            }
            class_counts[class_id]++;
            int obj_num = class_counts[class_id];

            std::string label = status + " - " + obj_name;
            if (obj_num > 1) {
                label += " #" + std::to_string(obj_num);
            }
            cv::putText(annotated, label, cv::Point(bx, by - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);

            // Kinematics text inside bbox bottom half
            cv::Scalar kin_color(0, 255, 255);
            int text_x = bx + 5;
            int text_y = by + bh / 2 + 20;

            char buf[64];
            std::snprintf(buf, sizeof(buf), "R: %.1f m", fd.range_m);
            cv::putText(annotated, buf, cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, kin_color, 1, cv::LINE_AA);

            std::snprintf(buf, sizeof(buf), "A: %.1f deg", fd.azimuth_deg);
            cv::putText(annotated, buf, cv::Point(text_x, text_y + 22),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, kin_color, 1, cv::LINE_AA);

            std::snprintf(buf, sizeof(buf), "Vr: %.2f m/s", fd.velocity_mps);
            cv::putText(annotated, buf, cv::Point(text_x, text_y + 44),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, kin_color, 1, cv::LINE_AA);

            // Velocity vector arrow
            if (show_velocity_vector_ && std::abs(fd.velocity_mps) > 0.05f) {
                float az_rad = fd.azimuth_deg * static_cast<float>(M_PI) / 180.0f;
                float end_x = fd.position_x_m + fd.velocity_mps * std::sin(az_rad) * 3.0f;
                float end_y = fd.position_y_m + fd.velocity_mps * std::cos(az_rad) * 3.0f;
                float end_z = fd.position_z_m;

                if (end_y > 0.1f) {
                    float end_u = CAMERA_FX * (end_x / end_y) + CAMERA_CX;
                    float end_v = CAMERA_FY * (-end_z / end_y) + CAMERA_CY;

                    if (end_u >= 0 && end_u < CAMERA_IMAGE_W &&
                        end_v >= 0 && end_v < CAMERA_IMAGE_H) {
                        cv::arrowedLine(annotated,
                                        cv::Point(static_cast<int>(draw_u), static_cast<int>(draw_v)),
                                        cv::Point(static_cast<int>(end_u), static_cast<int>(end_v)),
                                        color, 3, cv::LINE_AA, 0, 0.3);
                    }
                }
            }

            // Zone dwell timer: track time spent within 3m
            if (fd.range_m < 3.0f) {
                if (zone_entry_times_.find(cache_key) == zone_entry_times_.end()) {
                    zone_entry_times_[cache_key] = std::chrono::steady_clock::now();
                }
            } else {
                zone_entry_times_.erase(cache_key);
            }
        }
    }

    // Clean up stale cache entries (not seen for 2+ frames)
    for (auto it = detection_cache_.begin(); it != detection_cache_.end(); ) {
        if (seen_keys.find(it->first) == seen_keys.end()) {
            it->second.missed_frames++;
            if (it->second.missed_frames > 2) {
                zone_entry_times_.erase(it->first);
                it = detection_cache_.erase(it);
                continue;
            }
        }
        ++it;
    }

    // Prediction arc overlay — only for the closest fused detection's track
    if (show_prediction_arc_ && latest_fused_detections_ &&
        !latest_fused_detections_->detections.empty()) {
        // Find the closest FusedDetection by range
        float min_range = std::numeric_limits<float>::max();
        float closest_az = 0.0f;
        for (const auto& fd : latest_fused_detections_->detections) {
            if (fd.range_m < min_range) {
                min_range = fd.range_m;
                closest_az = fd.azimuth_deg;
            }
        }

        for (const auto& [track_id, traj] : latest_trajectories_) {
            size_t n = std::min(traj.waypoints_x_m.size(), static_cast<size_t>(8));
            if (n < 2) continue;

            // Only draw arc for the trajectory matching the closest detection
            float traj_az = std::atan2(static_cast<float>(traj.waypoints_x_m[0]),
                                       static_cast<float>(traj.waypoints_y_m[0]))
                            * 180.0f / static_cast<float>(M_PI);
            if (std::abs(traj_az - closest_az) >= 15.0f) continue;

            // Project waypoints to image space and draw polyline
            std::vector<cv::Point> arc_pts;
            for (size_t i = 0; i < n; ++i) {
                float z_cam = static_cast<float>(traj.waypoints_y_m[i]);
                if (z_cam <= 0.1f) continue;
                float x_cam = static_cast<float>(traj.waypoints_x_m[i]);
                float y_cam = -static_cast<float>(traj.waypoints_z_m[i]);
                float u = CAMERA_FX * (x_cam / z_cam) + CAMERA_CX;
                float v = CAMERA_FY * (y_cam / z_cam) + CAMERA_CY;

                if (u >= 0 && u < CAMERA_IMAGE_W && v >= 0 && v < CAMERA_IMAGE_H) {
                    arc_pts.emplace_back(static_cast<int>(u), static_cast<int>(v));

                    // Uncertainty ellipse (capped)
                    if (i < traj.uncertainty_radii_m.size()) {
                        float r_px = static_cast<float>(traj.uncertainty_radii_m[i])
                                     * CAMERA_FX / z_cam;
                        r_px = std::min(r_px, 30.0f);
                        if (r_px > 2.0f) {
                            cv::ellipse(annotated,
                                        cv::Point(static_cast<int>(u), static_cast<int>(v)),
                                        cv::Size(static_cast<int>(r_px), static_cast<int>(r_px * 0.6f)),
                                        0, 0, 360,
                                        cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
                        }
                    }
                }
            }

            if (arc_pts.size() >= 2) {
                cv::polylines(annotated, arc_pts, false, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
                cv::circle(annotated, arc_pts.back(), 8, cv::Scalar(0, 165, 255), cv::FILLED);
            }
            break; // Only draw for the one closest match
        }
    }

    // Zone dwell timer display
    if (show_zone_timer_) {
        double max_dwell = 0.0;
        auto now_time = std::chrono::steady_clock::now();
        for (const auto& [key, entry_time] : zone_entry_times_) {
            double dwell = std::chrono::duration<double>(now_time - entry_time).count();
            if (dwell > max_dwell) max_dwell = dwell;
        }
        if (max_dwell > 1.0) {
            char dwell_buf[32];
            std::snprintf(dwell_buf, sizeof(dwell_buf), "Zone: %.1fs", max_dwell);
            cv::putText(annotated, dwell_buf,
                        cv::Point(annotated.cols - 250, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }
    }

    // Track table overlay (top-left)
    if (show_track_table_ && latest_fused_detections_ &&
        !latest_fused_detections_->detections.empty()) {
        int num_rows = std::min(static_cast<int>(latest_fused_detections_->detections.size()), 8);
        int table_w = 320;
        int table_h = 20 + 18 * num_rows;

        // Semi-transparent black background
        cv::Mat roi = annotated(cv::Rect(0, 0, table_w, table_h));
        cv::Mat overlay(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
        cv::addWeighted(overlay, 0.6, roi, 0.4, 0, roi);

        int row = 0;
        for (const auto& fd : latest_fused_detections_->detections) {
            if (row >= 8) break;

            // Find threat level and quality for this detection
            std::string tl = "UNK";
            float quality = 0.0f;
            if (latest_threats_) {
                float best_dist = std::numeric_limits<float>::max();
                for (const auto& r : latest_threats_->reports) {
                    float dx = r.position_x_m - fd.position_x_m;
                    float dy = r.position_y_m - fd.position_y_m;
                    float dist = dx * dx + dy * dy;
                    if (dist < best_dist) {
                        best_dist = dist;
                        tl = r.threat_level;
                        quality = r.quality_score;
                    }
                }
            }

            // Shorten threat level
            std::string tl_short;
            if (tl == "BENIGN") tl_short = "BENIGN";
            else if (tl == "SUSPECT") tl_short = "SUSPCT";
            else if (tl == "THREAT") tl_short = "THREAT";
            else tl_short = "UNK";

            cv::Scalar tl_color;
            if (tl == "BENIGN")       tl_color = cv::Scalar(0, 255, 0);
            else if (tl == "SUSPECT") tl_color = cv::Scalar(0, 255, 255);
            else if (tl == "THREAT")  tl_color = cv::Scalar(0, 0, 255);
            else                      tl_color = cv::Scalar(200, 200, 200);

            char row_buf[80];
            std::snprintf(row_buf, sizeof(row_buf), "R:%.1fm Az:%.0fd",
                          fd.range_m, fd.azimuth_deg);

            int y = 16 + row * 18;
            cv::putText(annotated, row_buf, cv::Point(5, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            cv::putText(annotated, tl_short, cv::Point(175, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, tl_color, 1, cv::LINE_AA);

            char q_buf[16];
            std::snprintf(q_buf, sizeof(q_buf), "Q:%.2f", quality);
            cv::putText(annotated, q_buf, cv::Point(265, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            ++row;
        }
    }

    // PPI radar inset (220x220 px) bottom-right
    if (show_ppi_) {
        const int sz = 220;
        const int ox = annotated.cols - sz - 10;
        const int oy = annotated.rows - sz - 10;
        const float px_per_m = 20.0f;
        const int cx = ox + sz / 2;
        const int cy = oy + sz / 2;

        cv::rectangle(annotated, cv::Point(ox, oy),
                      cv::Point(ox + sz, oy + sz),
                      cv::Scalar(0, 0, 0), cv::FILLED);

        cv::Scalar blue(255, 100, 0);

        for (int r_m = 2; r_m <= 8; r_m += 2) {
            int r_px = static_cast<int>(r_m * px_per_m);
            cv::circle(annotated, cv::Point(cx, cy), r_px, blue, 1);
        }

        for (int deg = 0; deg < 360; deg += 30) {
            float rad = static_cast<float>(deg) * static_cast<float>(M_PI) / 180.0f;
            int lx = cx + static_cast<int>(100.0f * std::sin(rad));
            int ly = cy - static_cast<int>(100.0f * std::cos(rad));
            cv::line(annotated, cv::Point(cx, cy), cv::Point(lx, ly), blue, 1);
        }

        if (latest_fused_detections_) {
            for (const auto& fd : latest_fused_detections_->detections) {
                float rad = fd.azimuth_deg * static_cast<float>(M_PI) / 180.0f;
                int dx = static_cast<int>(fd.range_m * px_per_m * std::sin(rad));
                int dy = static_cast<int>(fd.range_m * px_per_m * std::cos(rad));
                int px = cx + dx;
                int py = cy - dy;

                if (px >= ox && px < ox + sz && py >= oy && py < oy + sz) {
                    cv::line(annotated, cv::Point(cx, cy), cv::Point(px, py), blue, 1);
                    cv::circle(annotated, cv::Point(px, py), 4,
                               cv::Scalar(255, 255, 255), cv::FILLED);
                }
            }
        }

        cv::putText(annotated, "RADAR", cv::Point(ox + 5, oy + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    auto out = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
    annotated_pub_->publish(*out);
}

CuasVisualizerNode::Color CuasVisualizerNode::threatColor(
    const std::string& level) const
{
    if (level == "BENIGN")  return {0.0, 1.0, 0.0, 0.8};
    if (level == "UNKNOWN") return {0.5, 0.5, 0.5, 0.8};
    if (level == "SUSPECT") return {1.0, 1.0, 0.0, 0.8};
    if (level == "THREAT")  return {1.0, 0.0, 0.0, 0.8};
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
    if (!latest_tracks_) return;

    const auto lifetime = rclcpp::Duration(0, 100000000);

    visualization_msgs::msg::MarkerArray track_markers;
    visualization_msgs::msg::MarkerArray traj_markers;
    visualization_msgs::msg::MarkerArray uncertainty_markers;
    visualization_msgs::msg::MarkerArray label_markers;

    for (const auto& track : latest_tracks_->tracks) {
        std::string threat = "BENIGN";
        auto it = threat_levels_.find(track.track_id);
        if (it != threat_levels_.end()) {
            threat = it->second;
        }
        Color tc = threatColor(threat);

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "base_link";
        sphere.header.stamp = this->now();
        sphere.ns = "track_positions";
        sphere.id = static_cast<int>(track.track_id);
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
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
        id_text.header.stamp = this->now();
        id_text.ns = "track_id_text";
        id_text.id = static_cast<int>(track.track_id);
        id_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        id_text.action = visualization_msgs::msg::Marker::ADD;
        id_text.pose.position.x = track.position_x_m;
        id_text.pose.position.y = track.position_y_m;
        id_text.pose.position.z = track.position_z_m + 0.5;
        id_text.pose.orientation.w = 1.0;
        id_text.scale.z = 0.25;
        id_text.color.r = 1.0;
        id_text.color.g = 1.0;
        id_text.color.b = 1.0;
        id_text.color.a = 1.0;
        id_text.lifetime = lifetime;

        double bearing = 0.0;
        double elevation = 0.0;
        auto pred_it = latest_predictions_.find(track.track_id);
        if (pred_it != latest_predictions_.end()) {
            bearing = pred_it->second.bearing_deg;
            elevation = pred_it->second.elevation_deg;
        } else {
            double px = track.position_x_m;
            double py = track.position_y_m;
            double pz = track.position_z_m;
            bearing = std::atan2(py, px) * 180.0 / M_PI;
            elevation = std::atan2(pz, std::sqrt(px * px + py * py)) * 180.0 / M_PI;
        }

        std::ostringstream oss;
        oss << "ID:" << track.track_id << " " << track.track_state << "\n"
            << std::fixed << std::setprecision(1)
            << bearing << "deg " << elevation << "deg";
        id_text.text = oss.str();
        label_markers.markers.push_back(id_text);

        auto traj_it = latest_trajectories_.find(track.track_id);
        if (traj_it != latest_trajectories_.end()) {
            const auto& traj = traj_it->second;
            Color pc = trackPaletteColor(track.track_id);
            size_t n = traj.waypoints_x_m.size();

            if (n > 0) {
                visualization_msgs::msg::Marker line;
                line.header.frame_id = "base_link";
                line.header.stamp = this->now();
                line.ns = "trajectories";
                line.id = static_cast<int>(track.track_id);
                line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line.action = visualization_msgs::msg::Marker::ADD;
                line.pose.orientation.w = 1.0;
                line.scale.x = 0.08;
                line.color.r = pc.r;
                line.color.g = pc.g;
                line.color.b = pc.b;
                line.color.a = pc.a;
                line.lifetime = lifetime;

                for (size_t i = 0; i < n; ++i) {
                    geometry_msgs::msg::Point pt;
                    pt.x = traj.waypoints_x_m[i];
                    pt.y = traj.waypoints_y_m[i];
                    pt.z = traj.waypoints_z_m[i];
                    line.points.push_back(pt);
                }
                traj_markers.markers.push_back(line);

                for (size_t i = 0; i < n; ++i) {
                    double radius = 0.1;
                    if (i < traj.uncertainty_radii_m.size()) {
                        radius = traj.uncertainty_radii_m[i];
                    }
                    radius = std::min(radius, 0.5);
                    radius = std::max(radius, 0.05);

                    double alpha = 0.3 - (static_cast<double>(i) / static_cast<double>(n > 1 ? n - 1 : 1)) * 0.25;
                    if (alpha < 0.05) alpha = 0.05;

                    visualization_msgs::msg::Marker usphere;
                    usphere.header.frame_id = "base_link";
                    usphere.header.stamp = this->now();
                    usphere.ns = "uncertainty_" + std::to_string(track.track_id);
                    usphere.id = static_cast<int>(i);
                    usphere.type = visualization_msgs::msg::Marker::SPHERE;
                    usphere.action = visualization_msgs::msg::Marker::ADD;
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::CuasVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
