cmake_minimum_required(VERSION 3.8)
project(point_cloud_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(point_cloud_sync src/main.cpp)
ament_target_dependencies(point_cloud_sync rclcpp sensor_msgs message_filters tf2_ros pcl_ros)

install(TARGETS
  point_cloud_sync
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
