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
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)

add_library(rscp_proto_messages
  proto/rscp.proto
)

set_target_properties(rscp_proto_messages PROPERTIES POSITION_INDEPENDENT_CODE ON)

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

set_target_properties(kalman_rscp_cobs PROPERTIES POSITION_INDEPENDENT_CODE ON)

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

add_library(rqt_rscp SHARED
  include/kalman_rscp/rqt_rscp.hpp
  src/rqt_rscp.cpp
)

set_target_properties(rqt_rscp PROPERTIES AUTOMOC ON)

target_include_directories(rqt_rscp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(rqt_rscp
  kalman_rscp_cobs
  rscp_proto_messages
  Qt5::Widgets
)

ament_target_dependencies(rqt_rscp
  pluginlib
  qt_gui_cpp
  rclcpp
  rqt_gui_cpp
  std_msgs
)

install(TARGETS
  kalman_rscp_cobs
  rscp_proto_messages
  rscp_node
  rqt_rscp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

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
