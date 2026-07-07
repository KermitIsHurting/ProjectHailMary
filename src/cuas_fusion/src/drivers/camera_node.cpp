// @file camera_node.cpp
// @brief ROS 2 node that publishes camera frames on a capture thread.
#include "cuas_fusion/drivers/camera_driver.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <cstdio>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

namespace cuas {

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_node"), running_(false)
    {
        pub_ = create_publisher<sensor_msgs::msg::Image>(CAMERA_TOPIC, 10);

        running_ = true;
        capture_thread_ = std::thread(&CameraNode::capture_loop, this);

        RCLCPP_INFO(get_logger(), "Camera node started — device: %s", CAMERA_DEVICE_PATH);
    }

    ~CameraNode() override
    {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        driver_.close();
    }

private:
    void capture_loop()
    {
        while (running_ && !driver_.open(CAMERA_DEVICE_PATH)) {
            RCLCPP_ERROR(get_logger(),
                "Failed to open camera %s — retrying in %d ms",
                CAMERA_DEVICE_PATH, CAMERA_RETRY_INTERVAL_MS);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(CAMERA_RETRY_INTERVAL_MS));
        }

        if (!running_) {
            return;
        }
        RCLCPP_INFO(get_logger(), "Camera opened successfully");

        cv::Mat bgr;
        int64_t ts_ns = 0;
        // Hoisted: a per-iteration Image allocated its ~6 MB data vector
        // every frame; resize() into retained capacity allocates once (R12e).
        sensor_msgs::msg::Image msg;

        while (running_) {
            if (!driver_.grabFrame(bgr, ts_ns)) {
                RCLCPP_ERROR(get_logger(),
                    "Frame grab failed — reopening device");
                driver_.close();

                while (running_ && !driver_.open(CAMERA_DEVICE_PATH)) {
                    RCLCPP_ERROR(get_logger(),
                        "Reopen failed — retrying in %d ms",
                        CAMERA_RETRY_INTERVAL_MS);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(CAMERA_RETRY_INTERVAL_MS));
                }
                continue;
            }

            msg.header.stamp.sec     = static_cast<int32_t>(ts_ns / 1'000'000'000LL);
            msg.header.stamp.nanosec = static_cast<uint32_t>(ts_ns % 1'000'000'000LL);
            msg.header.frame_id      = "camera";
            msg.height               = static_cast<uint32_t>(bgr.rows);
            msg.width                = static_cast<uint32_t>(bgr.cols);
            msg.encoding             = "bgr8";
            msg.is_bigendian         = false;
            msg.step                 = static_cast<uint32_t>(bgr.step);

            const size_t data_size = msg.step * msg.height;
            msg.data.resize(data_size);
            std::memcpy(msg.data.data(), bgr.data, data_size);

            pub_->publish(msg);
        }

        RCLCPP_INFO(get_logger(), "Capture thread exited");
    }

    CameraDriver  driver_;
    std::atomic<bool> running_;
    std::thread   capture_thread_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
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
        auto node = std::make_shared<cuas::CameraNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in CameraNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in CameraNode\n");
        exit_code = 1;
    }
    return exit_code;
}
