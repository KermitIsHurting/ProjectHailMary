// threat_classifier_node.cpp
// ROS 2 node entry point for threat classification: subscribes to TrackArray,
// runs ThreatClassifier on each confirmed track, and publishes ThreatReports.

#include "cuas_fusion/classification/threat_classifier.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cuas {

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
