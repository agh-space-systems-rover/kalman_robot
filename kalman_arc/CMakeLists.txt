cmake_minimum_required(VERSION 3.8)
project(kalman_arc)

# Suppress PCL_ROOT warning
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Read ROS distro
if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_HUMBLE)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(kalman_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)

add_library(rscp_proto_messages
  proto/rscp.proto
)

set_target_properties(rscp_proto_messages PROPERTIES POSITION_INDEPENDENT_CODE ON)

protobuf_generate(
  TARGET rscp_proto_messages
  LANGUAGE cpp
  PROTOS proto/rscp.proto
)

target_link_libraries(rscp_proto_messages
  protobuf::libprotobuf
)

target_include_directories(rscp_proto_messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

add_library(kalman_arc_cobs
  src/cobs.cpp
)

set_target_properties(kalman_arc_cobs PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(kalman_arc_cobs
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(rscp_node src/proto_node.cpp)

target_link_libraries(rscp_node
  kalman_arc_cobs
  rscp_proto_messages
)

ament_target_dependencies(rscp_node
  rclcpp
  std_msgs
  kalman_interfaces
)

add_library(rqt_rscp SHARED
  include/kalman_arc/rqt_rscp.hpp
  src/rqt_rscp.cpp
)

set_target_properties(rqt_rscp PROPERTIES AUTOMOC ON)

target_include_directories(rqt_rscp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(rqt_rscp
  kalman_arc_cobs
  rscp_proto_messages
  Qt5::Widgets
)

ament_target_dependencies(rqt_rscp
  pluginlib
  qt_gui_cpp
  rclcpp
  rqt_gui_cpp
  std_msgs
)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(peak_finder_node SHARED src/peak_finder_node.cpp)
ament_target_dependencies(peak_finder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  grid_map_ros
  grid_map_core
  grid_map_msgs
  geometry_msgs
)
rclcpp_components_register_node(
  peak_finder_node
  PLUGIN "kalman_arc::PeakFinder"
  EXECUTABLE peak_finder
)
install(TARGETS peak_finder_node
  EXPORT kalman_arc-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(PROGRAMS
  scripts/travel_distance_meter
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  kalman_arc_cobs
  rscp_proto_messages
  rscp_node
  rqt_rscp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(cobs_test test/cobs_test.cpp)
  target_link_libraries(cobs_test
    kalman_arc_cobs
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
