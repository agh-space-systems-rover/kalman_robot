// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "service_based_nav2_controller_srvs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace service_based_nav2_controller_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
cdr_serialize(
  const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
get_serialized_size(
  const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
max_serialized_size_ComputeVelocityCommands_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "service_based_nav2_controller_srvs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace service_based_nav2_controller_srvs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
cdr_serialize(
  const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
get_serialized_size(
  const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
max_serialized_size_ComputeVelocityCommands_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "service_based_nav2_controller_srvs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_service_based_nav2_controller_srvs
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands)();

#ifdef __cplusplus
}
#endif

#endif  // SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
