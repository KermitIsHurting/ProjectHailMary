// @file intent_classifier_node.cpp
// @brief ROS 2 node wrapping IntentClassifier for per-track behavioral intent.
#include "cuas_fusion/intent_classifier.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/intent_ids.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/intent_report.hpp>
#include <cuas_msgs/msg/intent_report_array.hpp>

#include <chrono>
#include <memory>
#include <cstdio>

namespace cuas {

class IntentClassifierNode : public rclcpp::Node
{
public:
    IntentClassifierNode()
    : Node("intent_classifier_node")
    {
        declare_parameter("publish_rate_hz", 10.0);
        // A rate above 1000 Hz truncated the period to 0 ms and spun the
        // timer; <= 0 was a division by zero (RC-14).
        const float64_t rate_hz = clamp_rate_hz(get_logger(), "publish_rate_hz",
            get_parameter("publish_rate_hz").as_double(), 10.0);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&IntentClassifierNode::track_callback,
                      this, std::placeholders::_1));

        pub_ = create_publisher<cuas_msgs::msg::IntentReportArray>(
            "/intent/reports", 10);

        const float64_t period_ms = 1000.0 / rate_hz;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
            std::bind(&IntentClassifierNode::publish_tick, this));

        RCLCPP_INFO(get_logger(),
            "Intent classifier node ready (rate=%.1fHz)", rate_hz);
    }

private:
    void track_callback(const cuas_msgs::msg::TrackArray::ConstSharedPtr & msg)
    {
        latest_tracks_ = msg;
    }

    void publish_tick()
    {
        if (latest_tracks_ == nullptr) {
            return;
        }

        cuas_msgs::msg::IntentReportArray out;
        out.stamp = this->now();

        const uint32_t n = static_cast<uint32_t>(latest_tracks_->tracks.size());
        for (uint32_t i = 0U; i < n; ++i) {
            const cuas_msgs::msg::Track & t = latest_tracks_->tracks[i];
            if ((t.track_state_id != cuas::track_state::kConfirmed) &&
                (t.track_state_id != cuas::track_state::kReacquired)) {
                continue;
            }

            IntentInput input;
            input.track_id  = t.track_id;
            input.x_m       = t.position_x_m;
            input.y_m       = t.position_y_m;
            input.vx_mps    = t.vx_mps;
            input.vy_mps    = t.vy_mps;
            input.speed_mps = t.velocity_mps;

            const IntentResult r = classifier_.classify(input);

            cuas_msgs::msg::IntentReport report;
            report.track_id          = t.track_id;
            report.intent            = r.intent;
            report.confidence        = r.confidence;
            report.loiter_radius_m   = r.loiter_radius_m;
            report.approach_rate_mps = r.approach_rate_mps;
            report.stamp             = out.stamp;

            out.reports.push_back(report);
        }

        pub_->publish(out);
    }

    IntentClassifier classifier_;
    cuas_msgs::msg::TrackArray::ConstSharedPtr latest_tracks_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
    rclcpp::Publisher<cuas_msgs::msg::IntentReportArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cuas

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
        auto node = std::make_shared<cuas::IntentClassifierNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in IntentClassifierNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in IntentClassifierNode\n");
        exit_code = 1;
    }
    return exit_code;
}
