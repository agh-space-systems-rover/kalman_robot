// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from kalman_interfaces:msg/GpsTag.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "kalman_interfaces/msg/detail/gps_tag__rosidl_typesupport_introspection_c.h"
#include "kalman_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "kalman_interfaces/msg/detail/gps_tag__functions.h"
#include "kalman_interfaces/msg/detail/gps_tag__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kalman_interfaces__msg__GpsTag__init(message_memory);
}

void kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_fini_function(void * message_memory)
{
  kalman_interfaces__msg__GpsTag__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_member_array[3] = {
  {
    "tag_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__msg__GpsTag, tag_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tag_lat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__msg__GpsTag, tag_lat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tag_lon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__msg__GpsTag, tag_lon),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_members = {
  "kalman_interfaces__msg",  // message namespace
  "GpsTag",  // message name
  3,  // number of fields
  sizeof(kalman_interfaces__msg__GpsTag),
  kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_member_array,  // message members
  kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_init_function,  // function to initialize message memory (memory has to be allocated)
  kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_type_support_handle = {
  0,
  &kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kalman_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, msg, GpsTag)() {
  if (!kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_type_support_handle.typesupport_identifier) {
    kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kalman_interfaces__msg__GpsTag__rosidl_typesupport_introspection_c__GpsTag_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
