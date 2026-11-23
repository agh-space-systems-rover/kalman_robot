cmake_minimum_required(VERSION 3.8)
project(kalman_arm_controller2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Core
add_library(kalman_can_core STATIC
  src/can/socket_can_interface.cpp
)
target_include_directories(kalman_can_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kalman_can_core PUBLIC
  Threads::Threads
)

# Gripper
add_library(kalman_gripper_protocol STATIC
  src/gripper/gripper_can_protocol.cpp
)
target_include_directories(kalman_gripper_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

add_executable(gripper_node
  src/gripper/gripper_node.cpp
  src/gripper/gripper_main.cpp
)
target_link_libraries(gripper_node
  kalman_can_core
  kalman_gripper_protocol
)
ament_target_dependencies(gripper_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
)

# Installation
install(TARGETS gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()