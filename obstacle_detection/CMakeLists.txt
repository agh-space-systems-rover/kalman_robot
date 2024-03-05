cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(obstacle_detection src/main.cpp)
ament_target_dependencies(obstacle_detection rclcpp sensor_msgs pcl_conversions pcl_ros tf2 tf2_ros tf2_geometry_msgs tf2_eigen)

install(TARGETS
  obstacle_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
