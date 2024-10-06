cmake_minimum_required(VERSION 3.8)
project(kalman_spacenav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/throttle_output
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
