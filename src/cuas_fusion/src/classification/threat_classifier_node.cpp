// @file threat_classifier_node.cpp
// @brief ROS 2 node wrapping ThreatClassifier and publishing threat reports.
#include "cuas_fusion/classification/threat_classifier.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/threat_report.hpp>
#include <cuas_msgs/msg/fused_detection_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <mutex>

namespace cuas {

class ClassifierNode : public rclcpp::Node
{
public:
    ClassifierNode()
    : Node("classifier_node")
    {
        if (!classifier_.init()) {
            RCLCPP_FATAL(get_logger(), "ThreatClassifier init failed");
            rclcpp::shutdown();
            return;
        }

        declare_parameter("threatening_range_m", 4.0);
        declare_parameter("threatening_velocity_mps", 0.3);
        declare_parameter("zone_radius_m", 3.0);
        declare_parameter("escalation_dwell_s", 1.0);
        declare_parameter("track_timeout_s", 5.0);

        threatening_range_m_ = static_cast<float>(get_parameter("threatening_range_m").as_double());
        threatening_velocity_mps_ = static_cast<float>(get_parameter("threatening_velocity_mps").as_double());
        zone_radius_m_ = static_cast<float>(get_parameter("zone_radius_m").as_double());
        escalation_dwell_s_ = static_cast<float>(get_parameter("escalation_dwell_s").as_double());
        track_timeout_s_ = get_parameter("track_timeout_s").as_double();

        pub_ = create_publisher<cuas_msgs::msg::ThreatReportArray>("/threat/reports", 5);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 5,
            std::bind(&ClassifierNode::trackCallback, this, std::placeholders::_1));

        fused_sub_ = create_subscription<cuas_msgs::msg::FusedDetectionArray>(
            "/fusion/detections", 5,
            std::bind(&ClassifierNode::fusedCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Classifier node ready (range=%.1f vel=%.1f zone=%.1f)",
                     static_cast<double>(threatening_range_m_),
                     static_cast<double>(threatening_velocity_mps_),
                     static_cast<double>(zone_radius_m_));
    }

private:
    void fusedCallback(const cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(fused_mutex_);
        latest_fused_ = msg;
    }

    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        double now_s = this->now().seconds();

        cuas_msgs::msg::ThreatReportArray out;
        out.header = msg->header;

        cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr fused;
        {
            std::lock_guard<std::mutex> lock(fused_mutex_);
            fused = latest_fused_;
        }

        for (const auto& tm : msg->tracks) {
            Track t;
            t.track_id_      = tm.track_id;
            t.position_x_m_  = tm.position_x_m;
            t.position_y_m_  = tm.position_y_m;
            t.position_z_m_  = tm.position_z_m;
            t.velocity_mps_  = tm.velocity_mps;
            t.doppler_mps_   = tm.doppler_mps;
            t.class_label_   = tm.class_label;
            t.confidence_    = tm.confidence;
            t.state_         = trackStateFromString(tm.track_state);
            t.timestamp_ns_  = tm.timestamp_ns;

            const cuas_msgs::msg::FusedDetection* matched_fd = nullptr;
            if (fused && !fused->detections.empty()) {
                float track_az = std::atan2(tm.position_x_m, tm.position_y_m)
                                 * 180.0f / static_cast<float>(M_PI);
                float best_diff = 999.0f;

                for (const auto& fd : fused->detections) {
                    float diff = std::abs(fd.azimuth_deg - track_az);
                    if (diff < best_diff) {
                        best_diff = diff;
                        matched_fd = &fd;
                    }
                }

                if (matched_fd && best_diff < 15.0f) {
                    t.class_label_ = matched_fd->class_label;
                    t.confidence_  = matched_fd->confidence;
                } else {
                    matched_fd = nullptr;
                }
            }

            ClassificationResult cr = classifier_.classify(
                t, now_s, threatening_range_m_, threatening_velocity_mps_, escalation_dwell_s_);

            if (!logged_first_ && !t.class_label_.empty()) {
                RCLCPP_INFO(get_logger(),
                    "First classification: track_id=%u class_label='%s' vel=%.2f -> %s esc=%s q=%.2f",
                    t.track_id_, t.class_label_.c_str(),
                    static_cast<double>(t.velocity_mps_),
                    threatLevelToString(cr.threat_level),
                    escalationStateToString(cr.escalation_state),
                    static_cast<double>(cr.quality_score));
                logged_first_ = true;
            }

            float az_rad = std::atan2(t.position_x_m_, t.position_y_m_);
            float pred_x = t.position_x_m_ + t.doppler_mps_ * std::sin(az_rad) * 5.0f;
            float pred_y = t.position_y_m_ + t.doppler_mps_ * std::cos(az_rad) * 5.0f;

            float range = std::sqrt(t.position_x_m_ * t.position_x_m_ +
                                    t.position_y_m_ * t.position_y_m_);

            cuas_msgs::msg::ThreatReport report;
            report.track_id            = t.track_id_;
            report.threat_level        = threatLevelToString(cr.threat_level);
            report.position_x_m        = t.position_x_m_;
            report.position_y_m        = t.position_y_m_;
            report.position_z_m        = t.position_z_m_;
            report.velocity_mps        = t.velocity_mps_;
            report.class_label         = t.class_label_;
            report.confidence          = t.confidence_;
            report.track_state         = tm.track_state;
            report.timestamp_ns        = t.timestamp_ns_;
            report.quality_score       = cr.quality_score;
            report.dwell_time_s        = cr.dwell_time_s;
            report.escalation_state    = escalationStateToString(cr.escalation_state);
            report.predicted_impact_x_m = pred_x;
            report.predicted_impact_y_m = pred_y;

            if (range < zone_radius_m_) {
                geometry_msgs::msg::Point zone_pt;
                zone_pt.x = 0.0;
                zone_pt.y = 0.0;
                zone_pt.z = 0.0;
                report.exclusion_zones_violated.push_back(zone_pt);
            }

            out.reports.push_back(report);
        }

        pub_->publish(out);

        classifier_.pruneStale(now_s, track_timeout_s_);
    }

    ThreatClassifier classifier_;
    rclcpp::Publisher<cuas_msgs::msg::ThreatReportArray>::SharedPtr pub_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
    rclcpp::Subscription<cuas_msgs::msg::FusedDetectionArray>::SharedPtr fused_sub_;

    std::mutex fused_mutex_;
    cuas_msgs::msg::FusedDetectionArray::ConstSharedPtr latest_fused_;
    bool logged_first_ = false;

    float threatening_range_m_ = 4.0f;
    float threatening_velocity_mps_ = 0.3f;
    float zone_radius_m_ = 3.0f;
    float escalation_dwell_s_ = 1.0f;
    double track_timeout_s_ = 5.0;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::ClassifierNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
