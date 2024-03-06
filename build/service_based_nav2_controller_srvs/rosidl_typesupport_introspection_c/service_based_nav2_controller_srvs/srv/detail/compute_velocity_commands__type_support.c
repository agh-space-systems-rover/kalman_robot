// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__rosidl_typesupport_introspection_c.h"
#include "service_based_nav2_controller_srvs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__functions.h"
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `velocity`
#include "geometry_msgs/msg/twist.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `path`
#include "nav_msgs/msg/path.h"
// Member `path`
#include "nav_msgs/msg/detail/path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__init(message_memory);
}

void service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_fini_function(void * message_memory)
{
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_member_array[3] = {
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request, path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_members = {
  "service_based_nav2_controller_srvs__srv",  // message namespace
  "ComputeVelocityCommands_Request",  // message name
  3,  // number of fields
  sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request),
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_member_array,  // message members
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_type_support_handle = {
  0,
  &service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_service_based_nav2_controller_srvs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Request)() {
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Path)();
  if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_type_support_handle.typesupport_identifier) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__rosidl_typesupport_introspection_c.h"
// already included above
// #include "service_based_nav2_controller_srvs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__functions.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.h"


// Include directives for member types
// Member `cmd_vel`
#include "geometry_msgs/msg/twist_stamped.h"
// Member `cmd_vel`
#include "geometry_msgs/msg/detail/twist_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__init(message_memory);
}

void service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_fini_function(void * message_memory)
{
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_member_array[1] = {
  {
    "cmd_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response, cmd_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_members = {
  "service_based_nav2_controller_srvs__srv",  // message namespace
  "ComputeVelocityCommands_Response",  // message name
  1,  // number of fields
  sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response),
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_member_array,  // message members
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_type_support_handle = {
  0,
  &service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_service_based_nav2_controller_srvs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Response)() {
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, TwistStamped)();
  if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_type_support_handle.typesupport_identifier) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "service_based_nav2_controller_srvs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_members = {
  "service_based_nav2_controller_srvs__srv",  // service namespace
  "ComputeVelocityCommands",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Request_message_type_support_handle,
  NULL  // response message
  // service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_Response_message_type_support_handle
};

static rosidl_service_type_support_t service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_type_support_handle = {
  0,
  &service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_service_based_nav2_controller_srvs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands)() {
  if (!service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_type_support_handle.typesupport_identifier) {
    service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Response)()->data;
  }

  return &service_based_nav2_controller_srvs__srv__detail__compute_velocity_commands__rosidl_typesupport_introspection_c__ComputeVelocityCommands_service_type_support_handle;
}
