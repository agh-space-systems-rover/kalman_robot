cmake_minimum_required(VERSION 3.8)
project(kalman_arch)

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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(record_video_node SHARED src/record_video_node.cpp)
target_link_libraries(record_video_node ${OpenCV_LIBS})
ament_target_dependencies(record_video_node rclcpp rclcpp_components std_msgs sensor_msgs image_transport cv_bridge std_srvs)
rclcpp_components_register_node(
  record_video_node
  PLUGIN "kalman_arch::RecordVideo"
  EXECUTABLE record_video
)

# add_library(cube_saver_node SHARED src/cube_saver_node.cpp)
# ament_target_dependencies(cube_saver_node rclcpp rclcpp_components std_msgs sensor_msgs image_transport cv_bridge vision_msgs tf2_ros tf2_geometry_msg)
# rclcpp_components_register_node(
#   cube_saver_node
#   PLUGIN "kalman_arch::DetectionSaver"
#   EXECUTABLE detection_saver
# )

ament_export_targets(kalman_arch-targets)

install(TARGETS record_video_node
  EXPORT kalman_arch-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# install(TARGETS cube_saver_node
#   EXPORT kalman_arch-targets
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
# )

install(PROGRAMS
  scripts/cube_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
