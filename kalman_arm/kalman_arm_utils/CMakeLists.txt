cmake_minimum_required(VERSION 3.8)
project(kalman_arm_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kalman_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/servo_param_setter
  scripts/trajectory_sender
  scripts/pose_request_sender
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(arm_state_publisher src/arm_state_publisher.cpp)
target_include_directories(arm_state_publisher PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(arm_state_publisher kalman_interfaces rclcpp sensor_msgs)
target_compile_features(arm_state_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS arm_state_publisher
DESTINATION lib/${PROJECT_NAME})  


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(kalman_interfaces rclcpp)
ament_package()
