cmake_minimum_required(VERSION 3.8)
project(kalman_arm_autonomy_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rosbag2_2026_08_26-00_51_05/
  DESTINATION share/${PROJECT_NAME}/rosbag2_2026_08_26-00_51_05
)

ament_package()
