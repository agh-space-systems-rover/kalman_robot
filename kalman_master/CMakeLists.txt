cmake_minimum_required(VERSION 3.8)
project(kalman_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kalman_interfaces REQUIRED)

add_library(arm_twist_driver_node SHARED src/arm_twist_driver_node.cpp)
ament_target_dependencies(arm_twist_driver_node rclcpp rclcpp_components geometry_msgs kalman_interfaces)
rclcpp_components_register_node(
  arm_twist_driver_node
  PLUGIN "kalman_master::ArmTwistDriver"
  EXECUTABLE arm_twist_driver
)

ament_export_targets(kalman_master-targets)
install(TARGETS arm_twist_driver_node
  EXPORT kalman_master-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/autonomy_switch_spam
  scripts/drill_driver
  scripts/feed_driver
  scripts/master_com
  scripts/master_loopback
  scripts/ros_link
  scripts/universal_driver
  scripts/tunnel_client
  scripts/ueuos_driver
  scripts/weight_driver
  scripts/wheel_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
