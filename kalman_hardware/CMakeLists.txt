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
