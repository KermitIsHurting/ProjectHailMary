cmake_minimum_required(VERSION 3.8)
project(cuas_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarDetection.msg"
  "msg/RadarFrame.msg"
  "msg/FusedDetection.msg"
  "msg/FusedDetectionArray.msg"
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "msg/ThreatReport.msg"
  "msg/ThreatReportArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
