cmake_minimum_required(VERSION 3.8)
project(service_based_nav2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(service_based_nav2_controller_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/main.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp nav2_core nav2_util nav_msgs pluginlib service_based_nav2_controller_srvs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core ${PROJECT_NAME}.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
