// @file cuas_color_correct_node.cpp
// @brief Subscribes to /camera/image_raw, applies per-channel BGR gain, republishes /camera/image_corrected.
#include "cuas_fusion/color_correct_engine.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstddef>
#include <memory>

namespace cuas {

class CuasColorCorrectNode : public rclcpp::Node
{
public:
    CuasColorCorrectNode()
    : Node("cuas_color_correct_node"),
      engine_()
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 5,
            std::bind(&CuasColorCorrectNode::imageCallback,
                      this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_corrected", 5);

        RCLCPP_INFO(get_logger(),
            "color correct ready — gains B=%.2f G=%.2f R=%.2f",
            static_cast<float64_t>(engine_.blue_gain()),
            static_cast<float64_t>(engine_.green_gain()),
            static_cast<float64_t>(engine_.red_gain()));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (msg->encoding != "bgr8") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "color correct expects bgr8, got '%s' — passing through",
                msg->encoding.c_str());
            pub_->publish(*msg);
            return;
        }

        const std::size_t pixel_count =
            static_cast<std::size_t>(msg->width) *
            static_cast<std::size_t>(msg->height);
        const std::size_t expected_bytes = pixel_count * 3U;

        if (msg->data.size() < expected_bytes) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "image buffer too small (have %zu, need %zu) — dropping frame",
                msg->data.size(), expected_bytes);
            return;
        }

        auto out = std::make_unique<sensor_msgs::msg::Image>(*msg);
        engine_.apply_bgr(out->data.data(), pixel_count);
        pub_->publish(std::move(out));
    }

    ColorCorrectEngine engine_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;
};

}  // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cuas::CuasColorCorrectNode>());
    rclcpp::shutdown();
    return 0;
}
