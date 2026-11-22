cmake_minimum_required(VERSION 3.8)
project(kalman_tunnel)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/master_com
  scripts/master_loopback
  scripts/ros_link
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # End-to-end ros_link test over master_loopback (no hardware needed).
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ros_link_loopback.py
    TIMEOUT 600
    ENV ROS_LOCALHOST_ONLY=1
  )

  # tunnel.launch.py in each mode, with master_com on a virtual serial port.
  add_launch_test(test/test_tunnel_launch.py
    TIMEOUT 300
    ENV ROS_LOCALHOST_ONLY=1
  )
endif()

ament_package()
