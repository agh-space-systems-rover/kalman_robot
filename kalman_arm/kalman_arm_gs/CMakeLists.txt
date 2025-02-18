cmake_minimum_required(VERSION 3.8)
project(kalman_arm_gs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kalman_interfaces REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(joint_republisher src/joint_republisher.cpp)
target_include_directories(joint_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(joint_republisher kalman_interfaces rclcpp)
target_compile_features(joint_republisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS joint_republisher
  DESTINATION lib/${PROJECT_NAME})


add_executable(gripper_republisher src/gripper_republisher.cpp)
target_include_directories(gripper_republisher PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(gripper_republisher kalman_interfaces rclcpp sensor_msgs)
target_compile_features(gripper_republisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS gripper_republisher
DESTINATION lib/${PROJECT_NAME})  


add_executable(arm_state_republisher src/arm_state_republisher.cpp)
target_include_directories(arm_state_republisher PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(arm_state_republisher kalman_interfaces rclcpp sensor_msgs)
target_compile_features(arm_state_republisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS arm_state_republisher
DESTINATION lib/${PROJECT_NAME}) 


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(kalman_interfaces rclcpp)
ament_package()
