cmake_minimum_required(VERSION 3.8)
project(kalman_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelState.msg"
  "msg/WheelStates.msg"
  "msg/GpsTag.msg"
  "msg/GpsTagArray.msg"
  "srv/SetUeuosColor.srv"
  "srv/SetUeuosEffect.srv"
  "srv/SetUeuosMode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
