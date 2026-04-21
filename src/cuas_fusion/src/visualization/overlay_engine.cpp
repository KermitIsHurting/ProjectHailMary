// @file overlay_engine.cpp
// @brief Implements pinhole projection and overlay drawing primitives.
#include "cuas_fusion/visualization/overlay_engine.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace cuas {

namespace {

static constexpr int32_t   kInvalidPx                = -1;
static constexpr int32_t   kThreatLabelYOffsetPx     = 24;
static constexpr int32_t   kConfirmedLabelThickness  = 2;
static constexpr int32_t   kTentativeLabelThickness  = 1;
static constexpr float32_t kFullLabelAlpha           = 1.0F;
static constexpr float64_t kArcAlphaDecayCoefficient = 0.7;
static constexpr float64_t kLabelBgBlendRatio        = 0.6;
static constexpr float64_t kLabelFgBlendRatio        = 0.4;
static constexpr float64_t kArcDotFarRadiusFactor    = 0.5;

constexpr int32_t clamp_int32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

// WHY: dot_radius = velocity_arrow_thickness * 3 clamped to [min, max]. All
// inputs are constexpr so the compiler folds this to a constant and the draw
// loop does no runtime arithmetic for the near radius.
static constexpr int32_t kArcDotNearRadiusPx =
    clamp_int32(kVelocityArrowThicknessPx * 3,
                kArcDotMinRadius, kArcDotMaxRadius);

ThreatLevel threat_level_from_string(const std::string& s)
{
    if (s == "BENIGN")  { return ThreatLevel::BENIGN;  }
    if (s == "SUSPECT") { return ThreatLevel::SUSPECT; }
    if (s == "THREAT")  { return ThreatLevel::THREAT;  }
    return ThreatLevel::UNKNOWN;
}

TrackState track_state_from_string(const std::string& s)
{
    if (s == "TENTATIVE")  { return TrackState::TENTATIVE;  }
    if (s == "CONFIRMED")  { return TrackState::CONFIRMED;  }
    if (s == "OCCLUDED")   { return TrackState::OCCLUDED;   }
    if (s == "REACQUIRED") { return TrackState::REACQUIRED; }
    if (s == "COASTED")    { return TrackState::COASTED;    }
    if (s == "LOST")       { return TrackState::LOST;       }
    if (s == "DELETED")    { return TrackState::DELETED;    }
    return TrackState::TENTATIVE;
}

}  // namespace

OverlayEngine::OverlayEngine(
    float64_t fx, float64_t fy, float64_t cx, float64_t cy,
    int32_t image_width, int32_t image_height)
: fx_(fx), fy_(fy), cx_(cx), cy_(cy),
  image_width_(image_width), image_height_(image_height)
{
}

cv::Point2i OverlayEngine::project_to_image(
    float64_t x_radar, float64_t y_radar, float64_t z_radar) const
{
    // Axis convention matches fusion_engine.cpp: radar (x-right, y-forward, z-up)
    // maps to camera (x-right, y-down, z-forward).
    const float64_t x_cam = x_radar;
    const float64_t z_cam = y_radar;
    const float64_t y_cam = -z_radar;

    if (z_cam <= 0.0) {
        return cv::Point2i(kInvalidPx, kInvalidPx);
    }

    const float64_t u = (fx_ * (x_cam / z_cam)) + cx_;
    const float64_t v = (fy_ * (y_cam / z_cam)) + cy_;

    return cv::Point2i(
        static_cast<int32_t>(std::lround(u)),
        static_cast<int32_t>(std::lround(v)));
}

bool OverlayEngine::is_in_bounds(const cv::Point2i& pt) const
{
    if ((pt.x < 0) || (pt.x >= image_width_)) {
        return false;
    }
    if ((pt.y < 0) || (pt.y >= image_height_)) {
        return false;
    }
    return true;
}

cv::Scalar OverlayEngine::threat_color(ThreatLevel level) const
{
    switch (level) {
        case ThreatLevel::BENIGN:  { return cv::Scalar(0.0,   255.0, 0.0);   }
        case ThreatLevel::UNKNOWN: { return cv::Scalar(255.0, 255.0, 255.0); }
        case ThreatLevel::SUSPECT: { return cv::Scalar(0.0,   255.0, 255.0); }
        case ThreatLevel::THREAT:  { return cv::Scalar(0.0,   0.0,   255.0); }
        default:                   { return cv::Scalar(255.0, 255.0, 255.0); }
    }
}

void OverlayEngine::draw_trajectory_arc(
    cv::Mat& image,
    const cuas_msgs::msg::TrajectoryWaypoints& wp,
    const cv::Scalar& color)
{
    FixedVector<cv::Point2i, static_cast<uint32_t>(kArcMaxWaypoints)> projected{};

    const std::size_t raw_count = std::min(
        std::min(wp.waypoints_x_m.size(), wp.waypoints_y_m.size()),
        wp.waypoints_z_m.size());
    const std::size_t count = std::min(
        raw_count, static_cast<std::size_t>(kArcMaxWaypoints));

    for (std::size_t i = 0U; i < count; ++i) {
        const cv::Point2i pt = project_to_image(
            wp.waypoints_x_m[i], wp.waypoints_y_m[i], wp.waypoints_z_m[i]);
        if ((pt.x == kInvalidPx) && (pt.y == kInvalidPx)) {
            break;
        }

        // WHY: stop the arc at the first waypoint inside the edge margin so a
        // long forecast does not exit the frame and re-enter, which reads as
        // a glitch. Skip-and-continue was tried and produced a visually
        // broken arc; hard break keeps the arc monotonic.
        const int32_t margin = kArcBoundsMarginPx;
        const bool in_margin =
            (pt.x >= margin) && (pt.x < (image_width_  - margin)) &&
            (pt.y >= margin) && (pt.y < (image_height_ - margin));
        if (!in_margin) {
            break;
        }

        (void)projected.push_back(pt);
    }

    if (projected.size() == 0U) {
        return;
    }

    uint32_t seg_val = 1U;
    if (projected.size() > 1U) {
        seg_val = projected.size() - 1U;
    }
    const uint32_t total_segments = seg_val;
    const float64_t seg_total = static_cast<float64_t>(total_segments);

    for (uint32_t i = 0U; (i + 1U) < projected.size(); ++i) {
        const float64_t alpha = 1.0 - ((static_cast<float64_t>(i) / seg_total) * kArcAlphaDecayCoefficient);
        const cv::Scalar blended(
            color[0] * alpha, color[1] * alpha, color[2] * alpha, color[3]);
        cv::line(image, projected[i], projected[i + 1U], blended, kArcLineThickness);
    }

    // WHY: nearest dot at full radius/opacity, furthest at half radius and
    // kArcDotMinAlpha opacity, intermediates linearly interpolated so the
    // forecast reads "fading away" rather than as a uniform string of dots.
    const float64_t dot_radius_near = static_cast<float64_t>(kArcDotNearRadiusPx);
    const float64_t dot_radius_far  = dot_radius_near * kArcDotFarRadiusFactor;
    const float64_t alpha_min       = static_cast<float64_t>(kArcDotMinAlpha);

    for (uint32_t i = 0U; i < projected.size(); ++i) {
        const float64_t t = static_cast<float64_t>(i) / seg_total;
        const float64_t radius_f = dot_radius_near + ((dot_radius_far - dot_radius_near) * t);
        const float64_t alpha    = 1.0 + ((alpha_min - 1.0) * t);
        const cv::Scalar blended(
            color[0] * alpha, color[1] * alpha, color[2] * alpha, color[3]);
        const int32_t radius_px = static_cast<int32_t>(std::lround(radius_f));
        cv::circle(image, projected[i], radius_px, blended, cv::FILLED);
    }
}

void OverlayEngine::draw_scaled_label(
    cv::Mat& image,
    const cv::Point2i& origin,
    const std::string& text,
    float32_t font_scale,
    int32_t thickness,
    const cv::Scalar& bg_color,
    const cv::Scalar& text_color,
    bool use_background,
    float32_t label_alpha)
{
    int32_t baseline = 0;
    const cv::Size text_size = cv::getTextSize(
        text, cv::FONT_HERSHEY_SIMPLEX,
        static_cast<float64_t>(font_scale), thickness, &baseline);

    const int32_t pad = kOverlayLabelPadding;
    const int32_t rect_x1 = std::max(0, origin.x - pad);
    const int32_t rect_y1 = std::max(0, origin.y - text_size.height - pad);
    const int32_t rect_x2 = std::min(image_width_ - 1, origin.x + text_size.width + pad);
    const int32_t rect_y2 = std::min(image_height_ - 1, origin.y + baseline + pad);

    if ((rect_x2 <= rect_x1) || (rect_y2 <= rect_y1)) {
        return;
    }

    const cv::Rect roi_rect(rect_x1, rect_y1, rect_x2 - rect_x1, rect_y2 - rect_y1);
    cv::Mat roi = image(roi_rect);

    if (use_background) {
        cv::Mat bg(roi.size(), roi.type(), bg_color);
        // Partial blend keeps the background context readable under the label.
        cv::addWeighted(bg, kLabelBgBlendRatio, roi, kLabelFgBlendRatio, 0.0, roi);
        cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX,
                    static_cast<float64_t>(font_scale), text_color, thickness);
    } else {
        // Compose text onto a copy of the ROI and blend back so unlabeled pixels are
        // unchanged while labeled pixels land at label_alpha opacity.
        cv::Mat roi_text_copy;
        roi.copyTo(roi_text_copy);
        const cv::Point2i roi_origin(origin.x - rect_x1, origin.y - rect_y1);
        cv::putText(roi_text_copy, text, roi_origin, cv::FONT_HERSHEY_SIMPLEX,
                    static_cast<float64_t>(font_scale), text_color, thickness);
        const float64_t alpha = static_cast<float64_t>(label_alpha);
        cv::addWeighted(roi_text_copy, alpha, roi, 1.0 - alpha, 0.0, roi);
    }
}

void OverlayEngine::render(
    cv::Mat& image,
    const FixedVector<cuas_msgs::msg::TrajectoryWaypoints, TRACK_MAX_TRACKS>& waypoints,
    const FixedVector<cuas_msgs::msg::Track, TRACK_MAX_TRACKS>& tracks,
    const FixedVector<cuas_msgs::msg::ThreatReport, TRACK_MAX_TRACKS>& threat_reports)
{
    const cv::Scalar white(255.0, 255.0, 255.0);

    for (uint32_t ti = 0U; ti < tracks.size(); ++ti) {
        const cuas_msgs::msg::Track& track = tracks[ti];

        ThreatLevel level = ThreatLevel::UNKNOWN;
        for (uint32_t ri = 0U; ri < threat_reports.size(); ++ri) {
            if (threat_reports[ri].track_id == track.track_id) {
                level = threat_level_from_string(threat_reports[ri].threat_level);
                break;
            }
        }
        const cv::Scalar color = threat_color(level);

        const TrackState state = track_state_from_string(track.track_state);
        const bool may_draw_arc =
            (state == TrackState::CONFIRMED) || (state == TrackState::REACQUIRED);

        if (may_draw_arc) {
            for (uint32_t wi = 0U; wi < waypoints.size(); ++wi) {
                if (waypoints[wi].track_id == track.track_id) {
                    draw_trajectory_arc(image, waypoints[wi], color);
                    break;
                }
            }
        }

        const cv::Point2i track_pt = project_to_image(
            static_cast<float64_t>(track.position_x_m),
            static_cast<float64_t>(track.position_y_m),
            static_cast<float64_t>(track.position_z_m));

        if ((track_pt.x == kInvalidPx) && (track_pt.y == kInvalidPx)) {
            continue;
        }
        if (!is_in_bounds(track_pt)) {
            continue;
        }

        const cv::Point2i threat_origin(track_pt.x, track_pt.y - kThreatLabelYOffsetPx);

        const cv::Scalar tentative_text_color(180.0, 180.0, 180.0);
        float32_t font_val = kOverlayTentativeFontScale;
        if (may_draw_arc) {
            font_val = kOverlayConfirmedFontScale;
        }
        const float32_t label_font_scale = font_val;
        int32_t thick_val = kTentativeLabelThickness;
        if (may_draw_arc) {
            thick_val = kConfirmedLabelThickness;
        }
        const int32_t label_thickness = thick_val;
        cv::Scalar label_text_color = tentative_text_color;
        if (may_draw_arc) {
            label_text_color = white;
        }
        const bool label_use_background = may_draw_arc;
        float32_t alpha_val = kOverlayTentativeAlpha;
        if (may_draw_arc) {
            alpha_val = kFullLabelAlpha;
        }
        const float32_t label_alpha = alpha_val;

        draw_scaled_label(image, threat_origin, track.class_label,
                          label_font_scale, label_thickness,
                          color, label_text_color,
                          label_use_background, label_alpha);

        std::ostringstream id_oss;
        id_oss << "T" << track.track_id;
        draw_scaled_label(image, track_pt, id_oss.str(),
                          label_font_scale, label_thickness,
                          color, label_text_color,
                          label_use_background, label_alpha);
    }
}

}  // namespace cuas
