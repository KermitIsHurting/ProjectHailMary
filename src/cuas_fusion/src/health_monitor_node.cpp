// @file health_monitor_node.cpp
// @brief ROS 2 node computing pipeline liveness from pipeline topic callbacks.
#include "cuas_fusion/common/fixed_types.hpp"
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
        const float64_t rate = get_parameter("publish_rate_hz").as_double();

        monitor_.set_expected_hz(kTopicRadar,      16.0F);
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
    float32_t now_sec() const
    {
        return static_cast<float32_t>(clock_->now().seconds());
    }

    void radar_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & /*msg*/)
    {
        monitor_.update(kTopicRadar, now_sec());
    }

    void camera_cb(const sensor_msgs::msg::Image::ConstSharedPtr & /*msg*/)
    {
        monitor_.update(kTopicCamera, now_sec());
    }

    void tracks_cb(const cuas_msgs::msg::TrackArray::ConstSharedPtr & /*msg*/)
    {
        monitor_.update(kTopicTracker, now_sec());
    }

    void threats_cb(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr & /*msg*/)
    {
        monitor_.update(kTopicClassifier, now_sec());
    }

    void predict_cb(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr & /*msg*/)
    {
        monitor_.update(kTopicPredictor, now_sec());
    }

    void publish_tick()
    {
        const float32_t t = now_sec();
        for (uint32_t i = 0U; i < kTopicCount; ++i) {
            monitor_.refresh_status(i, t);
        }

        const TopicHealth radar      = monitor_.query(kTopicRadar);
        const TopicHealth camera     = monitor_.query(kTopicCamera);
        const TopicHealth tracker    = monitor_.query(kTopicTracker);
        const TopicHealth classifier = monitor_.query(kTopicClassifier);
        const TopicHealth predictor  = monitor_.query(kTopicPredictor);

        cuas_msgs::msg::SystemHealth msg;
        msg.status            = monitor_.overall_status();
        msg.radar_status      = radar.status;
        msg.camera_status     = camera.status;
        msg.tracker_status    = tracker.status;
        msg.classifier_status = classifier.status;
        msg.predictor_status  = predictor.status;
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

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::HealthMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
