cmake_minimum_required(VERSION 3.8)
project(kalman_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Read ROS_DISTRO and if it is "humble", define ROS_HUMBLE.
if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_HUMBLE)
endif()

add_library(dead_reckoning_node SHARED src/dead_reckoning_node.cpp)
ament_target_dependencies(dead_reckoning_node rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(dead_reckoning_node ${OpenCV_LIBS})
rclcpp_components_register_node(
  dead_reckoning_node
  PLUGIN "kalman_slam::DeadReckoning"
  EXECUTABLE dead_reckoning
)

ament_export_targets(kalman_slam-targets)
install(TARGETS dead_reckoning_node
  EXPORT kalman_slam-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/fiducial_odometry
  scripts/gps_spoofer
  scripts/gps_preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config fiducials launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
