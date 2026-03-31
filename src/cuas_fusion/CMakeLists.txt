cmake_minimum_required(VERSION 3.8)
project(cuas_fusion)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cuas_msgs REQUIRED)

# radar parser node
add_executable(radar_parser_node
  src/drivers/radar_parser_node.cpp
)
ament_target_dependencies(radar_parser_node
  rclcpp
  sensor_msgs
  cuas_msgs
)

install(TARGETS
  radar_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
