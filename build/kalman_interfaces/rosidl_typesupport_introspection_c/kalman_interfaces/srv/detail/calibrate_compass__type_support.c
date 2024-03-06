// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "kalman_interfaces/srv/detail/calibrate_compass__rosidl_typesupport_introspection_c.h"
#include "kalman_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"
#include "kalman_interfaces/srv/detail/calibrate_compass__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kalman_interfaces__srv__CalibrateCompass_Request__init(message_memory);
}

void kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_fini_function(void * message_memory)
{
  kalman_interfaces__srv__CalibrateCompass_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_member_array[2] = {
  {
    "duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Request, duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Request, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_members = {
  "kalman_interfaces__srv",  // message namespace
  "CalibrateCompass_Request",  // message name
  2,  // number of fields
  sizeof(kalman_interfaces__srv__CalibrateCompass_Request),
  kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_member_array,  // message members
  kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_type_support_handle = {
  0,
  &kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kalman_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass_Request)() {
  if (!kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_type_support_handle.typesupport_identifier) {
    kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kalman_interfaces__srv__CalibrateCompass_Request__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kalman_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kalman_interfaces__srv__CalibrateCompass_Response__init(message_memory);
}

void kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_fini_function(void * message_memory)
{
  kalman_interfaces__srv__CalibrateCompass_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_member_array[14] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_mag_field",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_mag_field),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_offset0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_offset0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_offset1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_offset1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_offset2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_offset2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_gain0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_gain0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_gain1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_gain1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_gain2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_gain2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_t0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_t0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_t1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_t1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_t2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_t2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_t3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_t3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_t4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_t4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cc_t5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__srv__CalibrateCompass_Response, cc_t5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_members = {
  "kalman_interfaces__srv",  // message namespace
  "CalibrateCompass_Response",  // message name
  14,  // number of fields
  sizeof(kalman_interfaces__srv__CalibrateCompass_Response),
  kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_member_array,  // message members
  kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_type_support_handle = {
  0,
  &kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kalman_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass_Response)() {
  if (!kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_type_support_handle.typesupport_identifier) {
    kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kalman_interfaces__srv__CalibrateCompass_Response__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "kalman_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_members = {
  "kalman_interfaces__srv",  // service namespace
  "CalibrateCompass",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_Request_message_type_support_handle,
  NULL  // response message
  // kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_Response_message_type_support_handle
};

static rosidl_service_type_support_t kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_type_support_handle = {
  0,
  &kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kalman_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass)() {
  if (!kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_type_support_handle.typesupport_identifier) {
    kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, srv, CalibrateCompass_Response)()->data;
  }

  return &kalman_interfaces__srv__detail__calibrate_compass__rosidl_typesupport_introspection_c__CalibrateCompass_service_type_support_handle;
}
