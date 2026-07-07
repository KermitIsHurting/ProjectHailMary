cmake_minimum_required(VERSION 3.8)
project(cuas_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cuas_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu)
find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64)

include_directories(include)
# Third-party headers are adopted code: SYSTEM suppresses their diagnostics so
# the zero-warning policy (JPL P10) gates only code we own.
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# -----------------------------------------------------------------------------
# JSF AV C++ / MISRA C++:2023 enforcement flags
#
# Exceptions and RTTI disabled per JSF AV Rule 208 and MISRA Rule 18-4-1.
# Third-party frameworks (ROS 2, OpenCV, Eigen, TensorRT) are adopted code
# with documented deviations per MISRA Compliance:2020 Section 6.5. See
# docs/CODING_STANDARD.md for DEV-001 through DEV-005.
#
# DEVIATION DEV-001 (ROS 2 rclcpp): rclcpp propagates std::exception internally,
# therefore -fno-exceptions is NOT applied here. Application code contains no
# try/catch/throw (verified via scan 12d).
# DEVIATION DEV-001 (ROS 2 rclcpp): rclcpp uses dynamic_cast in its message
# template machinery, therefore -fno-rtti is NOT applied here. Application
# code contains no dynamic_cast/typeid (verified via scan 12c).
# -----------------------------------------------------------------------------
set(CUAS_SAFETY_FLAGS
    -Wall
    -Wextra
    -Wpedantic
    -Wcast-align
    -Wdouble-promotion
    -Wformat=2
    -Wnull-dereference
    -Wimplicit-fallthrough
    -Wswitch-enum
)

# Stricter set used only when the target has no direct ROS header dependency.
set(CUAS_STRICT_FLAGS
    ${CUAS_SAFETY_FLAGS}
    -Wsign-conversion
    -Wconversion
)

# radar parser node
add_executable(radar_parser_node
  src/drivers/radar_parser_node.cpp
)
ament_target_dependencies(radar_parser_node
  rclcpp
  sensor_msgs
  cuas_msgs
)
target_compile_options(radar_parser_node PRIVATE ${CUAS_SAFETY_FLAGS})

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
target_include_directories(camera_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})
target_link_libraries(camera_node ${OpenCV_LIBS} ${GST_LIBRARIES})
target_compile_options(camera_node PRIVATE ${CUAS_SAFETY_FLAGS})

# TRT detector library (no ROS dependency — use stricter flags)
add_library(trt_detector STATIC
  src/inference/trt_detector.cpp
)
target_include_directories(trt_detector PUBLIC include)
target_include_directories(trt_detector SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/cuda-12.6/targets/aarch64-linux/include
  /usr/local/cuda/include
)
target_link_libraries(trt_detector
  ${OpenCV_LIBS}
  ${NVINFER_LIB}
  ${CUDART_LIB}
)
target_compile_options(trt_detector PRIVATE ${CUAS_SAFETY_FLAGS})

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
target_compile_options(inference_node PRIVATE ${CUAS_SAFETY_FLAGS})

# fusion engine library (no ROS dependency)
add_library(fusion_engine STATIC
  src/fusion/fusion_engine.cpp
)
target_include_directories(fusion_engine PUBLIC include)
target_compile_options(fusion_engine PRIVATE ${CUAS_SAFETY_FLAGS})

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
target_compile_options(fusion_node PRIVATE ${CUAS_SAFETY_FLAGS})

# tracking library (no ROS dependency)
add_library(tracking_lib STATIC
  src/tracking/track_manager.cpp
  src/tracking/hungarian_solver.cpp
  src/common/types.cpp
)
target_include_directories(tracking_lib PUBLIC include)
target_compile_options(tracking_lib PRIVATE ${CUAS_SAFETY_FLAGS})

# tracker node
add_executable(tracker_node
  src/tracking/track_manager_node.cpp
)
ament_target_dependencies(tracker_node
  rclcpp
  cuas_msgs
)
target_link_libraries(tracker_node tracking_lib)
target_compile_options(tracker_node PRIVATE ${CUAS_SAFETY_FLAGS})

# classification library (no ROS dependency)
add_library(classification_lib STATIC
  src/classification/threat_classifier.cpp
)
target_include_directories(classification_lib PUBLIC include)
target_link_libraries(classification_lib tracking_lib)
target_compile_options(classification_lib PRIVATE ${CUAS_SAFETY_FLAGS})

# classifier node
add_executable(classifier_node
  src/classification/threat_classifier_node.cpp
)
ament_target_dependencies(classifier_node
  rclcpp
  cuas_msgs
)
target_link_libraries(classifier_node classification_lib)
target_compile_options(classifier_node PRIVATE ${CUAS_SAFETY_FLAGS})

# imm tracker node
add_executable(imm_tracker_node
  src/tracking/imm_tracker_node.cpp
  src/tracking/imm_tracker.cpp
  src/estimation/imm_filter.cpp
  src/estimation/kalman_cv.cpp
  src/estimation/kalman_ca.cpp
  src/estimation/kalman_ct.cpp
)
ament_target_dependencies(imm_tracker_node
  rclcpp sensor_msgs cuas_msgs geometry_msgs)
target_include_directories(imm_tracker_node SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIR})
target_compile_options(imm_tracker_node PRIVATE ${CUAS_SAFETY_FLAGS})

# kinematic predictor node
add_executable(kinematic_predictor_node
  src/prediction/kinematic_predictor_node.cpp
  src/prediction/kinematic_predictor.cpp
  src/estimation/imm_filter.cpp
  src/estimation/kalman_cv.cpp
  src/estimation/kalman_ca.cpp
  src/estimation/kalman_ct.cpp
)
ament_target_dependencies(kinematic_predictor_node
  rclcpp cuas_msgs geometry_msgs)
target_include_directories(kinematic_predictor_node SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIR})
target_compile_options(kinematic_predictor_node PRIVATE ${CUAS_SAFETY_FLAGS})

# occlusion predictor node
add_executable(occlusion_predictor_node
  src/prediction/occlusion_predictor_node.cpp
  src/prediction/occlusion_predictor.cpp
  src/prediction/kinematic_predictor.cpp
  src/estimation/imm_filter.cpp
  src/estimation/kalman_cv.cpp
  src/estimation/kalman_ca.cpp
  src/estimation/kalman_ct.cpp
)
ament_target_dependencies(occlusion_predictor_node
  rclcpp cuas_msgs geometry_msgs)
target_include_directories(occlusion_predictor_node SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIR})
target_compile_options(occlusion_predictor_node PRIVATE ${CUAS_SAFETY_FLAGS})

# prediction mux node
add_executable(prediction_mux_node
  src/prediction/prediction_mux_node.cpp
)
ament_target_dependencies(prediction_mux_node
  rclcpp cuas_msgs)
target_compile_options(prediction_mux_node PRIVATE ${CUAS_SAFETY_FLAGS})

# cot publisher node
add_executable(cot_publisher_node
  src/output/cot_publisher_node.cpp
)
ament_target_dependencies(cot_publisher_node
  rclcpp
  cuas_msgs
)
target_compile_options(cot_publisher_node PRIVATE ${CUAS_SAFETY_FLAGS})

# cuas visualizer node
add_executable(cuas_visualizer_node
  src/visualization/cuas_visualizer_node.cpp
)
ament_target_dependencies(cuas_visualizer_node
  rclcpp
  cuas_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  vision_msgs
  sensor_msgs
  cv_bridge
)
target_include_directories(cuas_visualizer_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cuas_visualizer_node ${OpenCV_LIBS})
target_compile_options(cuas_visualizer_node PRIVATE ${CUAS_SAFETY_FLAGS})

# cuas overlay node (downstream of visualizer; no cv_bridge — uses rosImageToBgr)
add_executable(cuas_overlay_node
  src/visualization/overlay_engine.cpp
  src/visualization/cuas_overlay_node.cpp
)
ament_target_dependencies(cuas_overlay_node
  rclcpp
  sensor_msgs
  cuas_msgs
)
target_include_directories(cuas_overlay_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cuas_overlay_node ${OpenCV_LIBS})
target_compile_options(cuas_overlay_node PRIVATE ${CUAS_SAFETY_FLAGS})

# capture node (test-only frame grabber, not launched by full.launch.py)
add_executable(capture_node
  src/visualization/capture_node.cpp
)
ament_target_dependencies(capture_node
  rclcpp
  sensor_msgs
)
target_include_directories(capture_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(capture_node ${OpenCV_LIBS})
target_compile_options(capture_node PRIVATE ${CUAS_SAFETY_FLAGS})

# geofence engine library (no ROS dependency)
add_library(geofence_engine STATIC
  src/geofence_engine.cpp
)
target_include_directories(geofence_engine PUBLIC include)
target_compile_options(geofence_engine PRIVATE ${CUAS_SAFETY_FLAGS})

# geofence node
add_executable(geofence_node
  src/geofence_node.cpp
)
ament_target_dependencies(geofence_node
  rclcpp
  cuas_msgs
)
target_link_libraries(geofence_node geofence_engine yaml-cpp)
target_compile_options(geofence_node PRIVATE ${CUAS_SAFETY_FLAGS})

# reachability engine library (no ROS dependency)
add_library(reachability_engine STATIC
  src/reachability_engine.cpp
)
target_include_directories(reachability_engine PUBLIC include)
target_compile_options(reachability_engine PRIVATE ${CUAS_SAFETY_FLAGS})

# reachability node
add_executable(reachability_node
  src/reachability_node.cpp
)
ament_target_dependencies(reachability_node
  rclcpp
  cuas_msgs
)
target_link_libraries(reachability_node reachability_engine)
target_compile_options(reachability_node PRIVATE ${CUAS_SAFETY_FLAGS})

# health monitor library (no ROS dependency — use stricter flags)
add_library(health_monitor_lib STATIC
  src/health_monitor.cpp
)
target_include_directories(health_monitor_lib PUBLIC include)
target_compile_options(health_monitor_lib PRIVATE ${CUAS_STRICT_FLAGS})

# health monitor node
add_executable(health_monitor_node
  src/health_monitor_node.cpp
)
ament_target_dependencies(health_monitor_node
  rclcpp
  sensor_msgs
  cuas_msgs
)
target_link_libraries(health_monitor_node health_monitor_lib)
target_compile_options(health_monitor_node PRIVATE ${CUAS_SAFETY_FLAGS})

# sim radar library (no ROS dependency — use stricter flags)
add_library(sim_radar_lib STATIC
  src/drivers/sim_radar.cpp
)
target_include_directories(sim_radar_lib PUBLIC include)
target_compile_options(sim_radar_lib PRIVATE ${CUAS_STRICT_FLAGS})

# sim radar node
add_executable(sim_radar_node
  src/drivers/sim_radar_node.cpp
)
ament_target_dependencies(sim_radar_node
  rclcpp
  sensor_msgs
)
target_link_libraries(sim_radar_node sim_radar_lib)
target_compile_options(sim_radar_node PRIVATE ${CUAS_SAFETY_FLAGS})

# clutter map library (no ROS dependency — use stricter flags)
add_library(clutter_map_lib STATIC
  src/clutter_map.cpp
)
target_include_directories(clutter_map_lib PUBLIC include)
target_compile_options(clutter_map_lib PRIVATE ${CUAS_STRICT_FLAGS})

# clutter map node
add_executable(clutter_map_node
  src/clutter_map_node.cpp
)
ament_target_dependencies(clutter_map_node
  rclcpp
  sensor_msgs
  cuas_msgs
)
target_link_libraries(clutter_map_node clutter_map_lib)
target_compile_options(clutter_map_node PRIVATE ${CUAS_SAFETY_FLAGS})

# intent classifier library (no ROS dependency — use stricter flags)
add_library(intent_classifier_lib STATIC
  src/intent_classifier.cpp
)
target_include_directories(intent_classifier_lib PUBLIC include)
target_compile_options(intent_classifier_lib PRIVATE ${CUAS_STRICT_FLAGS})

# intent classifier node
add_executable(intent_classifier_node
  src/intent_classifier_node.cpp
)
ament_target_dependencies(intent_classifier_node
  rclcpp
  cuas_msgs
)
target_link_libraries(intent_classifier_node intent_classifier_lib)
target_compile_options(intent_classifier_node PRIVATE ${CUAS_SAFETY_FLAGS})

# color correct engine library (no ROS dependency — use stricter flags)
add_library(color_correct_engine STATIC
  src/color_correct_engine.cpp
)
target_include_directories(color_correct_engine PUBLIC include)
target_compile_options(color_correct_engine PRIVATE ${CUAS_STRICT_FLAGS})

# color correct node
add_executable(cuas_color_correct_node
  src/cuas_color_correct_node.cpp
)
ament_target_dependencies(cuas_color_correct_node
  rclcpp
  sensor_msgs
)
target_link_libraries(cuas_color_correct_node color_correct_engine)
target_compile_options(cuas_color_correct_node PRIVATE ${CUAS_SAFETY_FLAGS})

install(TARGETS
  radar_parser_node
  camera_node
  inference_node
  fusion_node
  tracker_node
  classifier_node
  imm_tracker_node
  kinematic_predictor_node
  occlusion_predictor_node
  prediction_mux_node
  cuas_visualizer_node
  cuas_overlay_node
  capture_node
  cot_publisher_node
  geofence_node
  reachability_node
  health_monitor_node
  sim_radar_node
  clutter_map_node
  intent_classifier_node
  cuas_color_correct_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/send_radar_config.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_timestamp_associator
    ../../test/unit/test_timestamp_associator.cpp
    src/fusion/timestamp_associator.cpp
  )
  target_include_directories(test_timestamp_associator PRIVATE include)
  target_include_directories(test_timestamp_associator SYSTEM PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_timestamp_associator
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_fixed_containers
    ../../test/unit/test_fixed_containers.cpp
  )
  target_include_directories(test_fixed_containers PRIVATE include)

  ament_add_gtest(test_threat_classifier
    ../../test/unit/test_threat_classifier.cpp
    src/classification/threat_classifier.cpp
    src/common/types.cpp
  )
  target_include_directories(test_threat_classifier PRIVATE include)

  ament_add_gtest(test_geofence_engine
    test/test_geofence_engine.cpp
    src/geofence_engine.cpp)
  target_include_directories(test_geofence_engine PRIVATE include)

  ament_add_gtest(test_reachability_engine
    test/test_reachability_engine.cpp
    src/reachability_engine.cpp)
  target_include_directories(test_reachability_engine PRIVATE include)

  ament_add_gtest(test_health_monitor
    test/test_health_monitor.cpp
    src/health_monitor.cpp)
  target_include_directories(test_health_monitor PRIVATE include)

  ament_add_gtest(test_intent_classifier
    test/test_intent_classifier.cpp
    src/intent_classifier.cpp)
  target_include_directories(test_intent_classifier PRIVATE include)

  ament_add_gtest(test_clutter_map
    test/test_clutter_map.cpp
    src/clutter_map.cpp)
  target_include_directories(test_clutter_map PRIVATE include)

  ament_add_gtest(test_color_correct_engine
    test/test_color_correct_engine.cpp
    src/color_correct_engine.cpp)
  target_include_directories(test_color_correct_engine PRIVATE include)
endif()

ament_package()
