cmake_minimum_required(VERSION 3.8)
project(spacenav_to_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kalman_interfaces REQUIRED)

add_executable(spacenav_to_master src/spacenav_to_master.cpp)

target_include_directories(spacenav_to_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(spacenav_to_master rclcpp geometry_msgs kalman_interfaces)

target_compile_features(spacenav_to_master PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS spacenav_to_master
  DESTINATION lib/${PROJECT_NAME})


ament_package()
