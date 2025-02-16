cmake_minimum_required(VERSION 3.8)
project(kalman_gs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gs
  scripts/autonomy_config
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY node_project
#   DESTINATION share/${PROJECT_NAME}
#   REGEX "node_modules" EXCLUDE
# )
# Instead of symlinking every file in node_project, just symlink the whole directory:
install(
  CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/node_project ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/node_project)"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
