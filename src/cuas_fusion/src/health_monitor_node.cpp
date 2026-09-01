// @file health_monitor_node.cpp
// @brief ROS 2 node computing pipeline liveness from pipeline topic callbacks.
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/health_monitor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/system_health.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/track_array.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>

namespace cuas {

class HealthMonitorNode : public rclcpp::Node
{
public:
    HealthMonitorNode()
    : Node("health_monitor_node")
    , monitor_()
    , clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
    {
        (void)declare_parameter<float64_t>("publish_rate_hz", 1.0);
        float64_t rate = get_parameter("publish_rate_hz").as_double();
        rate = clamp_rate_hz(get_logger(), "publish_rate_hz", rate, 1.0);

        monitor_.set_expected_hz(kTopicRadar,      20.0F);
        monitor_.set_expected_hz(kTopicCamera,     30.0F);
        monitor_.set_expected_hz(kTopicTracker,    20.0F);
        monitor_.set_expected_hz(kTopicClassifier, 20.0F);
        monitor_.set_expected_hz(kTopicPredictor,  20.0F);

        sub_radar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10,
            std::bind(&HealthMonitorNode::radar_cb, this, std::placeholders::_1));

        sub_camera_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 1,
            std::bind(&HealthMonitorNode::camera_cb, this, std::placeholders::_1));

        sub_tracks_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&HealthMonitorNode::tracks_cb, this, std::placeholders::_1));

        sub_threats_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
            "/threat/reports", 10,
            std::bind(&HealthMonitorNode::threats_cb, this, std::placeholders::_1));

        sub_predict_ = create_subscription<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks", 10,
            std::bind(&HealthMonitorNode::predict_cb, this, std::placeholders::_1));

        pub_ = create_publisher<cuas_msgs::msg::SystemHealth>("/health/status", 10);

        const float64_t period_ms_d = 1000.0 / rate;
        const int32_t   period_ms   = static_cast<int32_t>(period_ms_d);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&HealthMonitorNode::publish_tick, this));

        RCLCPP_INFO(get_logger(),
                    "Health monitor node ready (rate=%.1fHz)", rate);
    }

private:
    int64_t now_ns() const
    {
        return clock_->now().nanoseconds();
    }

    void radar_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
    {
        (void)msg;
        monitor_.update(kTopicRadar, now_ns());
    }

    void camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        (void)msg;
        monitor_.update(kTopicCamera, now_ns());
    }

    void tracks_cb(const cuas_msgs::msg::TrackArray::ConstSharedPtr & msg)
    {
        (void)msg;
        monitor_.update(kTopicTracker, now_ns());
    }

    void threats_cb(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr & msg)
    {
        (void)msg;
        monitor_.update(kTopicClassifier, now_ns());
    }

    void predict_cb(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr & msg)
    {
        (void)msg;
        monitor_.update(kTopicPredictor, now_ns());
    }

    void publish_tick()
    {
        const int64_t t = now_ns();
        for (uint32_t i = 0U; i < kTopicCount; ++i) {
            monitor_.refresh_status(i, t);
        }

        const TopicHealth radar      = monitor_.query(kTopicRadar);
        const TopicHealth camera     = monitor_.query(kTopicCamera);
        const TopicHealth tracker    = monitor_.query(kTopicTracker);
        const TopicHealth classifier = monitor_.query(kTopicClassifier);
        const TopicHealth predictor  = monitor_.query(kTopicPredictor);

        cuas_msgs::msg::SystemHealth msg;
        msg.status            = static_cast<uint8_t>(monitor_.overall_status());
        msg.radar_status      = static_cast<uint8_t>(radar.status);
        msg.camera_status     = static_cast<uint8_t>(camera.status);
        msg.tracker_status    = static_cast<uint8_t>(tracker.status);
        msg.classifier_status = static_cast<uint8_t>(classifier.status);
        msg.predictor_status  = static_cast<uint8_t>(predictor.status);
        msg.radar_hz          = radar.measured_hz;
        msg.tracker_hz        = tracker.measured_hz;
        msg.classifier_hz     = classifier.measured_hz;
        msg.predictor_hz      = predictor.measured_hz;
        msg.stamp             = clock_->now();

        pub_->publish(msg);
    }

    HealthMonitor                       monitor_;
    std::shared_ptr<rclcpp::Clock>      clock_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr         sub_radar_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr               sub_camera_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr            sub_tracks_;
    rclcpp::Subscription<cuas_msgs::msg::ThreatReportArray>::SharedPtr     sub_threats_;
    rclcpp::Subscription<cuas_msgs::msg::PredictedTrack>::SharedPtr        sub_predict_;

    rclcpp::Publisher<cuas_msgs::msg::SystemHealth>::SharedPtr             pub_;
    rclcpp::TimerBase::SharedPtr                                           timer_;
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
        auto node = std::make_shared<cuas::HealthMonitorNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in HealthMonitorNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in HealthMonitorNode\n");
        exit_code = 1;
    }
    return exit_code;
}
