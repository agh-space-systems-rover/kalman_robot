// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__STRUCT_H_
#define SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in srv/ComputeVelocityCommands in the package service_based_nav2_controller_srvs.
typedef struct service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request
{
  geometry_msgs__msg__PoseStamped pose;
  geometry_msgs__msg__Twist velocity;
  nav_msgs__msg__Path path;
} service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request;

// Struct for a sequence of service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request.
typedef struct service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence
{
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cmd_vel'
#include "geometry_msgs/msg/detail/twist_stamped__struct.h"

/// Struct defined in srv/ComputeVelocityCommands in the package service_based_nav2_controller_srvs.
typedef struct service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response
{
  geometry_msgs__msg__TwistStamped cmd_vel;
} service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response;

// Struct for a sequence of service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response.
typedef struct service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence
{
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__STRUCT_H_
