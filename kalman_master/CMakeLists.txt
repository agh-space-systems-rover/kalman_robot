cmake_minimum_required(VERSION 3.8)
project(kalman_master)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/autonomy_switch_spam
  scripts/drill_driver
  scripts/feed_driver
  scripts/master_com
  scripts/master_loopback
  scripts/ros_link
  scripts/tunnel_client
  scripts/ueuos_driver
  scripts/wheel_driver
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
