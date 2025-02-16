cmake_minimum_required(VERSION 3.8)
project(kalman_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SupervisorGpsArUcoSearch.action"
  "action/SupervisorGpsGoal.action"
  "action/SupervisorGpsYoloSearch.action"
  "action/SupervisorTfGoal.action"
  "msg/ArmAxesLocks.msg"
  "msg/ArmCompressed.msg"
  "msg/ArmFkCommand.msg"
  "msg/ArmGoalStatus.msg"
  "msg/ArmPoseSelect.msg"
  "msg/ArmState.msg"
  "msg/ArmTrajectorySelect.msg"
  "msg/Drill.msg"
  "msg/Drive.msg"
  "msg/MasterMessage.msg"
  "msg/WheelState.msg"
  "msg/WheelStates.msg"
  "msg/WheelTemperature.msg"
  "msg/WheelTemperatures.msg"
  "msg/AutonomyStatus.msg"
  "msg/Device.msg"
  "srv/GetDevices.srv"
  "srv/SetFeed.srv"
  "srv/SetUeuosColor.srv"
  "srv/SetUeuosEffect.srv"
  "srv/SetUeuosState.srv"
  "srv/SpoofGps.srv"
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
