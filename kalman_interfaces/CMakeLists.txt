cmake_minimum_required(VERSION 3.8)
project(kalman_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpsTagArray.msg"
  "msg/GpsTag.msg"
  "msg/MasterMessage.msg"
  "msg/WheelState.msg"
  "msg/WheelStates.msg"
  "srv/CalibrateCompass.srv"
  "srv/SetUeuosColor.srv"
  "srv/SetUeuosEffect.srv"
  "srv/SetUeuosMode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
