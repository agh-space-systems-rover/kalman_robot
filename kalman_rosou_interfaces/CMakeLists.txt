cmake_minimum_required(VERSION 3.8)
project(bat_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CompressedCostmap.msg
  msg/Configuration.msg
  msg/GpsTagArray.msg
  msg/GpsTag.msg
)

ament_package()
