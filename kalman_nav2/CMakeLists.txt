cmake_minimum_required(VERSION 3.8)
project(kalman_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/geo_path_converter
  scripts/path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
