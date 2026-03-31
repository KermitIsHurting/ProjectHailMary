// track_manager_node.cpp
// ROS 2 node entry point for track management: consumes FusedDetections,
// drives TrackManager, and publishes the confirmed TrackArray at each cycle.

#include "cuas_fusion/tracking/track_manager.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cuas {

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
