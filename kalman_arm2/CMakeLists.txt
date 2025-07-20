cmake_minimum_required(VERSION 3.8)
project(kalman_arm2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(kalman_interfaces REQUIRED)

add_library(joint_republisher_node SHARED src/joint_republisher_node.cpp)
ament_target_dependencies(joint_republisher_node rclcpp rclcpp_components kalman_interfaces sensor_msgs)
rclcpp_components_register_node(
  joint_republisher_node
  PLUGIN "kalman_arm2::JointRepublisher"
  EXECUTABLE joint_republisher
)

add_library(twist_ik_node SHARED src/twist_ik_node.cpp)
ament_target_dependencies(twist_ik_node rclcpp rclcpp_components kalman_interfaces geometry_msgs std_msgs urdf kdl_parser orocos_kdl_vendor tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(
  twist_ik_node
  PLUGIN "kalman_arm2::TwistIK"
  EXECUTABLE twist_ik
)

add_library(gamepad_control_node SHARED src/gamepad_control_node.cpp)
ament_target_dependencies(gamepad_control_node rclcpp rclcpp_components sensor_msgs geometry_msgs kalman_interfaces)
rclcpp_components_register_node(
  gamepad_control_node
  PLUGIN "kalman_arm2::GamepadControl"
  EXECUTABLE gamepad_control
)

add_library(goto_joint_pose_node SHARED src/goto_joint_pose_node.cpp)
ament_target_dependencies(goto_joint_pose_node rclcpp rclcpp_components kalman_interfaces geometry_msgs std_msgs tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(
  goto_joint_pose_node
  PLUGIN "kalman_arm2::GotoJointPose"
  EXECUTABLE goto_joint_pose
)


ament_export_targets(kalman_arm2-targets)
install(TARGETS joint_republisher_node twist_ik_node gamepad_control_node goto_joint_pose_node
  EXPORT kalman_arm2-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
