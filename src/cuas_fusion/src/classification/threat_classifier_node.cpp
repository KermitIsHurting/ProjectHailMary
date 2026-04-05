#include "cuas_fusion/classification/threat_classifier.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/tracking/track.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/threat_report.hpp>

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

        pub_ = create_publisher<cuas_msgs::msg::ThreatReportArray>("/threat/reports", 5);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks/confirmed", 5,
            std::bind(&ClassifierNode::trackCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Classifier node ready");
    }

private:
    void trackCallback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        cuas_msgs::msg::ThreatReportArray out;
        out.header = msg->header;

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
            t.state_         = (tm.track_state == "CONFIRMED") ? TrackState::CONFIRMED
                                                                : TrackState::TENTATIVE;
            t.timestamp_ns_  = tm.timestamp_ns;

            ThreatLevel level = classifier_.classify(t);

            cuas_msgs::msg::ThreatReport report;
            report.track_id     = t.track_id_;
            report.threat_level = threatLevelToString(level);
            report.position_x_m = t.position_x_m_;
            report.position_y_m = t.position_y_m_;
            report.position_z_m = t.position_z_m_;
            report.velocity_mps = t.velocity_mps_;
            report.class_label  = t.class_label_;
            report.confidence   = t.confidence_;
            report.track_state  = tm.track_state;
            report.timestamp_ns = t.timestamp_ns_;
            out.reports.push_back(report);
        }

        pub_->publish(out);
    }

    ThreatClassifier classifier_;
    rclcpp::Publisher<cuas_msgs::msg::ThreatReportArray>::SharedPtr pub_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
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
