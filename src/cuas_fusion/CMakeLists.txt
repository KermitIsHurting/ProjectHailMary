cmake_minimum_required(VERSION 3.8)
project(cuas_fusion)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cuas_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu)
find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64)

include_directories(include)

# radar parser node
add_executable(radar_parser_node
  src/drivers/radar_parser_node.cpp
)
ament_target_dependencies(radar_parser_node
  rclcpp
  sensor_msgs
  cuas_msgs
)

# camera node
add_executable(camera_node
  src/drivers/camera_driver.cpp
  src/drivers/camera_node.cpp
)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_node ${OpenCV_LIBS})

# TRT detector library (no ROS dependency)
add_library(trt_detector STATIC
  src/inference/trt_detector.cpp
)
target_include_directories(trt_detector PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/cuda-12.6/targets/aarch64-linux/include
  /usr/local/cuda/include
)
target_link_libraries(trt_detector
  ${OpenCV_LIBS}
  ${NVINFER_LIB}
  ${CUDART_LIB}
)

# inference node
add_executable(inference_node
  src/inference/inference_node.cpp
)
ament_target_dependencies(inference_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
)
target_link_libraries(inference_node trt_detector)

# fusion engine library (no ROS dependency)
add_library(fusion_engine STATIC
  src/fusion/fusion_engine.cpp
)
target_include_directories(fusion_engine PUBLIC include)

# fusion node
add_executable(fusion_node
  src/fusion/fusion_node.cpp
)
ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  vision_msgs
  cuas_msgs
  tf2_ros
  geometry_msgs
)
target_link_libraries(fusion_node fusion_engine)

# tracking library (no ROS dependency)
add_library(tracking_lib STATIC
  src/tracking/track_manager.cpp
  src/common/types.cpp
)
target_include_directories(tracking_lib PUBLIC include)

# tracker node
add_executable(tracker_node
  src/tracking/track_manager_node.cpp
)
ament_target_dependencies(tracker_node
  rclcpp
  cuas_msgs
)
target_link_libraries(tracker_node tracking_lib)

# classification library (no ROS dependency)
add_library(classification_lib STATIC
  src/classification/threat_classifier.cpp
)
target_include_directories(classification_lib PUBLIC include)
target_link_libraries(classification_lib tracking_lib)

# classifier node
add_executable(classifier_node
  src/classification/threat_classifier_node.cpp
)
ament_target_dependencies(classifier_node
  rclcpp
  cuas_msgs
)
target_link_libraries(classifier_node classification_lib)

install(TARGETS
  radar_parser_node
  camera_node
  inference_node
  fusion_node
  tracker_node
  classifier_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_timestamp_associator
    ../../test/unit/test_timestamp_associator.cpp
    src/fusion/timestamp_associator.cpp
  )
  target_include_directories(test_timestamp_associator PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_timestamp_associator
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_threat_classifier
    ../../test/unit/test_threat_classifier.cpp
    src/classification/threat_classifier.cpp
    src/common/types.cpp
  )
  target_include_directories(test_threat_classifier PRIVATE include)
endif()

ament_package()
