cmake_minimum_required(VERSION 3.8)
project(kalman_arch)

# Suppress PCL_ROOT warning
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
# Read ROS distro
if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_HUMBLE)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cob_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(record_video_node SHARED src/record_video_node.cpp)
target_link_libraries(record_video_node ${OpenCV_LIBS})
ament_target_dependencies(record_video_node rclcpp rclcpp_components std_msgs sensor_msgs image_transport cv_bridge cob_srvs)
rclcpp_components_register_node(
  record_video_node
  PLUGIN "kalman_arch::RecordVideo"
  EXECUTABLE record_video
)

add_library(slam_serialization_node SHARED src/slam_serialization_node.cpp)
target_link_libraries(slam_serialization_node yaml-cpp)
ament_target_dependencies(slam_serialization_node rclcpp rclcpp_components std_msgs sensor_msgs nav_msgs PCL pcl_ros pcl_conversions geometry_msgs yaml-cpp)
rclcpp_components_register_node(
  slam_serialization_node
  PLUGIN "kalman_arch::SlamSerialization"
  EXECUTABLE slam_serialization
)


ament_export_targets(kalman_arch-targets)

install(TARGETS record_video_node
  EXPORT kalman_arch-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS slam_serialization_node
  EXPORT kalman_arch-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)


install(PROGRAMS
  scripts/cube_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz tools
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
