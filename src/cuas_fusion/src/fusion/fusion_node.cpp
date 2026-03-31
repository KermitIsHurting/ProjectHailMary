// fusion_node.cpp
// ROS 2 node entry point for the fusion pipeline: subscribes to radar and
// camera detection topics, drives FusionEngine, and publishes FusedDetections.

#include "cuas_fusion/fusion/fusion_engine.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cuas {

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
