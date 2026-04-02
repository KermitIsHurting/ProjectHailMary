// camera_node.cpp
// ROS 2 node wrapper for CameraDriver: spins a capture thread, demosaics
// Bayer frames, and publishes sensor_msgs/Image to /camera/image_raw.
// Constructs Image messages directly to avoid cv_bridge OpenCV version issues.

#include "cuas_fusion/drivers/camera_driver.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
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
        // Open with retry
        while (running_ && !driver_.open(CAMERA_DEVICE_PATH)) {
            RCLCPP_ERROR(get_logger(),
                "Failed to open camera %s — retrying in %d ms",
                CAMERA_DEVICE_PATH, CAMERA_RETRY_INTERVAL_MS);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(CAMERA_RETRY_INTERVAL_MS));
        }

        if (!running_) return;
        RCLCPP_INFO(get_logger(), "Camera opened successfully");

        cv::Mat bgr;
        int64_t ts_ns = 0;

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

            // Build sensor_msgs::msg::Image directly (no cv_bridge needed)
            sensor_msgs::msg::Image msg;
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

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
