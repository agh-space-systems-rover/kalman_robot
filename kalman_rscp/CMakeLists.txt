cmake_minimum_required(VERSION 3.8)
project(kalman_rscp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(kalman_interfaces REQUIRED)

add_library(rscp_proto_messages
  proto/rscp.proto
)

protobuf_generate(
  TARGET rscp_proto_messages
  LANGUAGE cpp
  PROTOS proto/rscp.proto
)

target_link_libraries(rscp_proto_messages
  protobuf::libprotobuf
)

target_include_directories(rscp_proto_messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

add_library(kalman_rscp_cobs
  src/cobs.cpp
)

target_include_directories(kalman_rscp_cobs
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(rscp_node src/proto_node.cpp)

target_link_libraries(rscp_node
  kalman_rscp_cobs
  rscp_proto_messages
)

ament_target_dependencies(rscp_node
  rclcpp
  std_msgs
  kalman_interfaces
)

install(TARGETS
  kalman_rscp_cobs
  rscp_proto_messages
  rscp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(cobs_test test/cobs_test.cpp)
  target_link_libraries(cobs_test
    kalman_rscp_cobs
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the COBS decoder is kept verbatim from the upstream source.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
