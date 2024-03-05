cmake_minimum_required(VERSION 3.8)
project(point_cloud_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(voxel_grid src/voxel_grid.cpp)
ament_target_dependencies(voxel_grid rclcpp sensor_msgs pcl_conversions pcl_ros)

add_executable(statistical_outlier_removal src/statistical_outlier_removal.cpp)
ament_target_dependencies(statistical_outlier_removal rclcpp sensor_msgs pcl_conversions pcl_ros)

install(TARGETS
  voxel_grid
  statistical_outlier_removal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
