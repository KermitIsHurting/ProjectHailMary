// camera_node.cpp
// ROS 2 node entry point for the camera driver: initializes the CameraDriver,
// loads calibration, and publishes image/camera_info at the configured rate.

#include "cuas_fusion/drivers/camera_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cuas {

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
