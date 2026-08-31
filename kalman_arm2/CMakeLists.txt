cmake_minimum_required(VERSION 3.8)
project(kalman_arm2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(kalman_interfaces REQUIRED)
set (CMAKE_CXX_STANDARD 20 REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(twist_ik_node SHARED src/twist_ik_node.cpp)
ament_target_dependencies(twist_ik_node rclcpp rclcpp_components kalman_interfaces geometry_msgs std_msgs urdf kdl_parser orocos_kdl_vendor tf2_ros tf2_geometry_msgs)
target_link_libraries(twist_ik_node Eigen3::Eigen)
rclcpp_components_register_node(
  twist_ik_node
  PLUGIN "kalman_arm2::TwistIK"
  EXECUTABLE twist_ik
)

add_library(pose_ik_node SHARED src/pose_ik_node.cpp)
ament_target_dependencies(pose_ik_node rclcpp rclcpp_components kalman_interfaces geometry_msgs std_msgs urdf kdl_parser orocos_kdl_vendor tf2_ros tf2_geometry_msgs)
target_link_libraries(pose_ik_node Eigen3::Eigen)
rclcpp_components_register_node(
  pose_ik_node
  PLUGIN "kalman_arm2::PoseIK"
  EXECUTABLE pose_ik
)

add_library(goto_joint_pose_node SHARED src/goto_joint_pose_node.cpp)
ament_target_dependencies(goto_joint_pose_node rclcpp rclcpp_components kalman_interfaces geometry_msgs std_msgs urdf tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(
  goto_joint_pose_node
  PLUGIN "kalman_arm2::GotoJointPose"
  EXECUTABLE goto_joint_pose
)

add_library(panel_layout_node SHARED src/panel_layout_node.cpp)
ament_target_dependencies(panel_layout_node rclcpp rclcpp_components kalman_interfaces geometry_msgs std_msgs tf2_ros tf2_geometry_msgs aruco_opencv_msgs yaml-cpp)
rclcpp_components_register_node(
  panel_layout_node
  PLUGIN "kalman_arm2::PanelLayout"
  EXECUTABLE panel_layout
)
target_link_libraries(panel_layout_node yaml-cpp)

add_library(panel_tracker_node SHARED src/panel_tracker_node.cpp)
ament_target_dependencies(panel_tracker_node rclcpp rclcpp_components geometry_msgs tf2_ros tf2_geometry_msgs aruco_opencv_msgs)
rclcpp_components_register_node(
  panel_tracker_node
  PLUGIN "kalman_arm2::PanelTracker"
  EXECUTABLE panel_tracker
)

add_library(panel_rectifier_node SHARED src/panel_rectifier_node.cpp)
ament_target_dependencies(panel_rectifier_node rclcpp rclcpp_components sensor_msgs tf2_ros tf2_geometry_msgs cv_bridge message_filters yaml-cpp kalman_interfaces vision_msgs)
target_include_directories(panel_rectifier_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(panel_rectifier_node ${OpenCV_LIBS} yaml-cpp)
rclcpp_components_register_node(
  panel_rectifier_node
  PLUGIN "kalman_arm2::PanelRectifier"
  EXECUTABLE panel_rectifier
)

add_library(panel_click_marker_node SHARED src/panel_click_marker_node.cpp)
ament_target_dependencies(panel_click_marker_node rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs visualization_msgs cv_bridge message_filters)
target_include_directories(panel_click_marker_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(panel_click_marker_node ${OpenCV_LIBS})
rclcpp_components_register_node(
  panel_click_marker_node
  PLUGIN "kalman_arm2::PanelClickMarker"
  EXECUTABLE panel_click_marker
)

add_library(bt_panel_node SHARED src/bt_panel_node.cpp
src/actions/do_something.cpp include/actions/do_something.hpp
src/actions/arm_navigate_to_pose.cpp include/actions/arm_navigate_to_pose.hpp
src/conditions/is_recent_detection.cpp include/conditions/is_recent_detection.hpp
src/actions/come_closer.cpp include/actions/come_closer.hpp
src/actions/average_pose.cpp include/actions/average_pose.hpp
src/mission_state.cpp include/mission_state.hpp
src/conditions/has_next_goal.cpp include/conditions/has_next_goal.hpp
src/actions/get_next_goal.cpp include/actions/get_next_goal.hpp
src/actions/ik_navigate_to_pose.cpp include/actions/ik_navigate_to_pose.hpp
src/actions/pose_ik_navigate_to_pose.cpp include/actions/pose_ik_navigate_to_pose.hpp
src/actions/set_mission_feedback.cpp include/actions/set_mission_feedback.hpp
src/actions/say_something.cpp include/actions/say_something.hpp
src/actions/build_uv.cpp include/actions/build_uv.hpp
src/actions/show_board.cpp include/actions/show_board.hpp
src/actions/acquire_panel_pose.cpp include/actions/acquire_panel_pose.hpp
src/actions/find_panel.cpp include/actions/find_panel.hpp
src/actions/compute_panel_target.cpp include/actions/compute_panel_target.hpp
src/conditions/is_panel_pose_available.cpp include/conditions/is_panel_pose_available.hpp
src/conditions/is_panel_pose_fresh.cpp include/conditions/is_panel_pose_fresh.hpp
src/conditions/is_panel_pose_good.cpp include/conditions/is_panel_pose_good.hpp
src/actions/visual_refine_to_panel.cpp include/actions/visual_refine_to_panel.hpp
src/actions/visual_refine_to_panel_twist.cpp include/actions/visual_refine_to_panel_twist.hpp
src/actions/gripper_actions.cpp include/actions/gripper_actions.hpp
src/actions/rotate_ee_along_normal_twist.cpp include/actions/rotate_ee_along_normal_twist.hpp
src/actions/rotate_ee_along_normal_joint.cpp include/actions/rotate_ee_along_normal_joint.hpp
src/actions/transform_pose.cpp include/actions/transform_pose.hpp
)
ament_target_dependencies(bt_panel_node rclcpp rclcpp_components kalman_interfaces geometry_msgs sensor_msgs std_msgs tf2_ros tf2_geometry_msgs aruco_opencv_msgs behaviortree_cpp_v3 std_srvs tf2_eigen visualization_msgs yaml-cpp)
rclcpp_components_register_node(
  bt_panel_node
  PLUGIN "kalman_arm2::BTPanel"
  EXECUTABLE bt_panel
)

target_link_libraries(bt_panel_node yaml-cpp)
# target_compile_features(bt_panel_node PUBLIC cxx_std_20)

ament_export_targets(kalman_arm2-targets)
install(TARGETS twist_ik_node pose_ik_node goto_joint_pose_node panel_layout_node panel_tracker_node panel_rectifier_node panel_click_marker_node bt_panel_node
  EXPORT kalman_arm2-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/arm1_moveit_compat
  scripts/gamepad_control
  scripts/joint_republisher
  scripts/rosbridge_client
  scripts/fake_yolo_publisher
  scripts/publish_identity_homography
  scripts/publish_fake_panel_target
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
