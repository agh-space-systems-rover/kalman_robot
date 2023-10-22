cmake_minimum_required(VERSION 3.8)
project(kalman_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include"
include_directories(include)

# Create Cpp executable
# add_executable(kalman_base src/kalman_base.cpp)
# ament_target_dependencies(kalman_base rclcpp)

## FOR ANOTHER FILES, CREATE NEW EXECUTABLES AND DEPENDENCIES HERE

# Install Cpp executable
# install(TARGETS 
#   kalman_base
#   # ADD MORE EXECUTABLES HERE
#   DESTINATION lib/${PROJECT_NAME})





# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/kalman_base.py
#   # ADD MORE SCRIPTS HERE
#   DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
