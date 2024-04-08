cmake_minimum_required(VERSION 3.8)
project(point_cloud_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(cloud_sync_node SHARED src/cloud_sync_node.cpp)
ament_target_dependencies(cloud_sync_node rclcpp rclcpp_components sensor_msgs message_filters tf2_ros pcl_ros)
rclcpp_components_register_node(
  cloud_sync_node
  PLUGIN "point_cloud_utils::CloudSync"
  EXECUTABLE cloud_sync
)

add_library(obstacle_detection_node SHARED src/obstacle_detection_node.cpp)
ament_target_dependencies(obstacle_detection_node rclcpp rclcpp_components sensor_msgs pcl_conversions pcl_ros tf2_ros tf2_eigen)
rclcpp_components_register_node(
  obstacle_detection_node
  PLUGIN "point_cloud_utils::ObstacleDetection"
  EXECUTABLE obstacle_detection
)

add_library(radius_outlier_removal_node SHARED src/radius_outlier_removal_node.cpp)
ament_target_dependencies(radius_outlier_removal_node rclcpp rclcpp_components sensor_msgs pcl_conversions pcl_ros)
rclcpp_components_register_node(
  radius_outlier_removal_node
  PLUGIN "point_cloud_utils::RadiusOutlierRemoval"
  EXECUTABLE radius_outlier_removal
)

add_library(rgbd_cloud_node SHARED src/rgbd_cloud_node.cpp)
ament_target_dependencies(rgbd_cloud_node rclcpp rclcpp_components sensor_msgs message_filters image_transport cv_bridge)
rclcpp_components_register_node(
  rgbd_cloud_node
  PLUGIN "point_cloud_utils::RgbdCloud"
  EXECUTABLE rgbd_cloud
)

add_library(statistical_outlier_removal_node SHARED src/statistical_outlier_removal_node.cpp)
ament_target_dependencies(statistical_outlier_removal_node rclcpp rclcpp_components sensor_msgs pcl_conversions pcl_ros)
rclcpp_components_register_node(
  statistical_outlier_removal_node
  PLUGIN "point_cloud_utils::StatisticalOutlierRemoval"
  EXECUTABLE statistical_outlier_removal
)

add_library(voxel_grid_node SHARED src/voxel_grid_node.cpp)
ament_target_dependencies(voxel_grid_node rclcpp rclcpp_components sensor_msgs pcl_conversions pcl_ros)
rclcpp_components_register_node(
  voxel_grid_node
  PLUGIN "point_cloud_utils::VoxelGrid"
  EXECUTABLE voxel_grid
)

ament_export_targets(point_cloud_utils-targets)
install(TARGETS cloud_sync_node obstacle_detection_node radius_outlier_removal_node rgbd_cloud_node statistical_outlier_removal_node voxel_grid_node
  EXPORT point_cloud_utils-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
