cmake_minimum_required(VERSION 3.16)
project(kalman_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    kalman_interfaces
    example_interfaces
    trajectory_msgs
    yaml_cpp_vendor)

set(TWIST_REPUB_DEPS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    builtin_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${TWIST_REPUB_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Component Nodes (Shared libraries) ############################
set(SERVO_PUBLISHER servo_publisher)
add_library(${SERVO_PUBLISHER} SHARED scripts/servo_publisher.cpp)
target_compile_features(${SERVO_PUBLISHER} PUBLIC cxx_std_20)
ament_target_dependencies(${SERVO_PUBLISHER} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_PUBLISHER} "arm_master::MasterToServo")

set(EXTRA_CAN extra_can)
add_library(${EXTRA_CAN} SHARED 
    scripts/extra_can_node.cpp
    hardware/can_src/can_driver.cpp
    hardware/can_src/arm_config.cpp
    hardware/can_src/can_driver.cpp
    hardware/can_src/can_handlers.cpp
    hardware/can_src/can_vars.cpp
)
target_compile_features(${EXTRA_CAN} PUBLIC cxx_std_20)
ament_target_dependencies(${EXTRA_CAN} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${EXTRA_CAN} "kalman_arm::ExtraCanNode")


add_executable(servo_keyboard_input scripts/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

set(CAN_BRIDGE can_bridge)
add_library(${CAN_BRIDGE} SHARED src/configurable_can_bridge.cpp)
target_compile_features(${CAN_BRIDGE} PUBLIC cxx_std_20)
ament_target_dependencies(${CAN_BRIDGE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${CAN_BRIDGE} "configurable_can_bridge::CANBridge")

# add_executable(monitor_planning_scene scripts/planning_scene_tutorial.cpp)
# target_include_directories(monitor_planning_scene PUBLIC include)
# ament_target_dependencies(monitor_planning_scene ${THIS_PACKAGE_INCLUDE_DEPENDS})

include_directories(hardware/include)
include_directories(include)


add_library(
  kalman_arm_controller
  SHARED
  hardware/arm_hardware.cpp
  hardware/can_src/arm_config.cpp
  hardware/can_src/can_driver.cpp
  hardware/can_src/can_handlers.cpp
  hardware/can_src/can_vars.cpp
)

target_compile_features(kalman_arm_controller PUBLIC cxx_std_20)
target_compile_options(kalman_arm_controller PRIVATE -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=clang++)
target_include_directories(kalman_arm_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/kalman_arm_controller>
)
ament_target_dependencies(
  kalman_arm_controller PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KALMAN_ARM_CONTROLLER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kalman_arm_controller.xml)

# INSTALL
install(
  DIRECTORY hardware/include/ include/
  DESTINATION include/kalman_arm_controller
)

install(TARGETS servo_keyboard_input
    DESTINATION lib/${PROJECT_NAME})

# install(TARGETS monitor_planning_scene
# DESTINATION lib/${PROJECT_NAME})

install(TARGETS kalman_arm_controller
    ${SERVO_PUBLISHER}
    ${EXTRA_CAN}
    ${CAN_BRIDGE}
  EXPORT export_kalman_arm_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_kalman_arm_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
