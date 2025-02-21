cmake_minimum_required(VERSION 3.8)
project(kalman_cubes)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(record_video_node SHARED src/record_video_node.cpp)
target_link_libraries(record_video_node ${OpenCV_LIBS})

ament_target_dependencies(record_video_node rclcpp rclcpp_components std_msgs sensor_msgs image_transport cv_bridge)
rclcpp_components_register_node(
  record_video_node
  PLUGIN "kalman_cubes::RecordVideo"
  EXECUTABLE record_video
)
ament_export_targets(kalman_cubes-targets)
install(TARGETS record_video_node
  EXPORT kalman_cubes-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)


install(PROGRAMS
  scripts/detections
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
