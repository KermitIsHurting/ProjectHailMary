// @file capture_node.cpp
// @brief Test utility that periodically saves /camera/annotated_enhanced frames as PNGs.
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/ros_image_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <cstdio>

namespace cuas {

static constexpr int32_t   kCaptureMaxFrames       = 20;
static constexpr float64_t kCaptureIntervalDefault = 3.0;
static constexpr float64_t kNsPerSecond            = 1.0e9;

class CaptureNode : public rclcpp::Node
{
public:
    CaptureNode()
    : Node("capture_node"),
      has_new_frame_(false),
      capture_count_(0),
      capture_interval_s_(kCaptureIntervalDefault)
    {
        declare_parameter<float64_t>("capture_interval_s", kCaptureIntervalDefault);
        declare_parameter<std::string>("capture_output_dir", "/home/zork/captures");

        capture_interval_s_ = get_parameter("capture_interval_s").as_double();
        // A zero/negative or NaN interval would busy-spin the timer; clamp
        // to a sane floor before deriving the period (negated: catches NaN).
        if (!(capture_interval_s_ >= 0.1)) {
            RCLCPP_WARN(get_logger(),
                        "capture_interval_s=%.3f invalid; using %.1f s",
                        capture_interval_s_, kCaptureIntervalDefault);
            capture_interval_s_ = kCaptureIntervalDefault;
        }
        output_dir_         = get_parameter("capture_output_dir").as_string();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/annotated_enhanced", 5,
            std::bind(&CaptureNode::imageCallback, this, std::placeholders::_1));

        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(capture_interval_s_ * kNsPerSecond));
        timer_ = create_wall_timer(period_ns,
            std::bind(&CaptureNode::timerTick, this));

        RCLCPP_INFO(get_logger(), "Capture node ready: interval=%.2fs dir='%s'",
                    capture_interval_s_, output_dir_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv::Mat bgr;
        if (!rosImageToBgr(*msg, bgr)) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_  = bgr.clone();
        has_new_frame_ = true;
    }

    void timerTick()
    {
        cv::Mat frame;
        int32_t idx = 0;
        bool    save = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (has_new_frame_ && !latest_frame_.empty()) {
                frame          = latest_frame_.clone();
                idx            = capture_count_;
                has_new_frame_ = false;
                save           = true;
            }
        }

        if (!save) {
            return;
        }

        std::ostringstream oss;
        oss << output_dir_ << "/frame_"
            << std::setw(4) << std::setfill('0') << idx << ".png";
        const std::string path = oss.str();

        // PNG backend reports errors via bool return, not exceptions.
        const bool ok = cv::imwrite(path, frame);
        if (ok) {
            RCLCPP_INFO(get_logger(), "Saved %s", path.c_str());
            ++capture_count_;
        } else {
            RCLCPP_WARN(get_logger(), "Failed to save %s", path.c_str());
        }

        if (capture_count_ >= kCaptureMaxFrames) {
            timer_->cancel();
            RCLCPP_INFO(get_logger(), "Capture complete: %d frames saved", capture_count_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr                             timer_;

    std::mutex mutex_;
    cv::Mat    latest_frame_;
    bool       has_new_frame_;
    int32_t    capture_count_;
    float64_t  capture_interval_s_;
    std::string output_dir_;
};

}  // namespace cuas

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
        auto node = std::make_shared<cuas::CaptureNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in CaptureNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in CaptureNode\n");
        exit_code = 1;
    }
    return exit_code;
}
