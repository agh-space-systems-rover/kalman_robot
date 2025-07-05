cmake_minimum_required(VERSION 3.8)
project(kalman_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(libphidget22 REQUIRED)

file(GLOB_RECURSE COMPASSCAL_SOURCES CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/compasscal_src/*.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/compasscal_src/*.c"
)

add_executable(compasscal ${COMPASSCAL_SOURCES}) 
ament_target_dependencies(compasscal libphidget22)
target_link_libraries(compasscal m)
target_compile_options(compasscal PRIVATE -w)

install(TARGETS compasscal
  DESTINATION lib/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)

# Read ROS_DISTRO and if it is "humble", define ROS_HUMBLE.
if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_HUMBLE)
endif()

add_library(rgbd_filter_node SHARED src/rgbd_filter_node.cpp)
ament_target_dependencies(rgbd_filter_node rclcpp rclcpp_components sensor_msgs image_transport message_filters cv_bridge)
target_link_libraries(rgbd_filter_node ${OpenCV_LIBS})
rclcpp_components_register_node(
  rgbd_filter_node
  PLUGIN "kalman_hardware::RgbdFilter"
  EXECUTABLE rgbd_filter
)

ament_export_targets(kalman_hardware-targets)
install(TARGETS rgbd_filter_node
  EXPORT kalman_hardware-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/compass_calibration
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
