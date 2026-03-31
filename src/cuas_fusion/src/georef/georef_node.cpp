// georef_node.cpp
// ROS 2 node entry point for georeferencing: subscribes to TrackArray,
// applies Wgs84Transform, and republishes tracks with geodetic coordinates.

#include "cuas_fusion/georef/wgs84_transform.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cuas {

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
