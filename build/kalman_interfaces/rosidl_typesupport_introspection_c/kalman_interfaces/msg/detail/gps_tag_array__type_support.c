// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "kalman_interfaces/msg/detail/gps_tag_array__rosidl_typesupport_introspection_c.h"
#include "kalman_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "kalman_interfaces/msg/detail/gps_tag_array__functions.h"
#include "kalman_interfaces/msg/detail/gps_tag_array__struct.h"


// Include directives for member types
// Member `tags`
#include "kalman_interfaces/msg/gps_tag.h"
// Member `tags`
#include "kalman_interfaces/msg/detail/gps_tag__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kalman_interfaces__msg__GpsTagArray__init(message_memory);
}

void kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_fini_function(void * message_memory)
{
  kalman_interfaces__msg__GpsTagArray__fini(message_memory);
}

size_t kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__size_function__GpsTagArray__tags(
  const void * untyped_member)
{
  const kalman_interfaces__msg__GpsTag__Sequence * member =
    (const kalman_interfaces__msg__GpsTag__Sequence *)(untyped_member);
  return member->size;
}

const void * kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__get_const_function__GpsTagArray__tags(
  const void * untyped_member, size_t index)
{
  const kalman_interfaces__msg__GpsTag__Sequence * member =
    (const kalman_interfaces__msg__GpsTag__Sequence *)(untyped_member);
  return &member->data[index];
}

void * kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__get_function__GpsTagArray__tags(
  void * untyped_member, size_t index)
{
  kalman_interfaces__msg__GpsTag__Sequence * member =
    (kalman_interfaces__msg__GpsTag__Sequence *)(untyped_member);
  return &member->data[index];
}

void kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__fetch_function__GpsTagArray__tags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const kalman_interfaces__msg__GpsTag * item =
    ((const kalman_interfaces__msg__GpsTag *)
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__get_const_function__GpsTagArray__tags(untyped_member, index));
  kalman_interfaces__msg__GpsTag * value =
    (kalman_interfaces__msg__GpsTag *)(untyped_value);
  *value = *item;
}

void kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__assign_function__GpsTagArray__tags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  kalman_interfaces__msg__GpsTag * item =
    ((kalman_interfaces__msg__GpsTag *)
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__get_function__GpsTagArray__tags(untyped_member, index));
  const kalman_interfaces__msg__GpsTag * value =
    (const kalman_interfaces__msg__GpsTag *)(untyped_value);
  *item = *value;
}

bool kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__resize_function__GpsTagArray__tags(
  void * untyped_member, size_t size)
{
  kalman_interfaces__msg__GpsTag__Sequence * member =
    (kalman_interfaces__msg__GpsTag__Sequence *)(untyped_member);
  kalman_interfaces__msg__GpsTag__Sequence__fini(member);
  return kalman_interfaces__msg__GpsTag__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_member_array[1] = {
  {
    "tags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces__msg__GpsTagArray, tags),  // bytes offset in struct
    NULL,  // default value
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__size_function__GpsTagArray__tags,  // size() function pointer
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__get_const_function__GpsTagArray__tags,  // get_const(index) function pointer
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__get_function__GpsTagArray__tags,  // get(index) function pointer
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__fetch_function__GpsTagArray__tags,  // fetch(index, &value) function pointer
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__assign_function__GpsTagArray__tags,  // assign(index, value) function pointer
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__resize_function__GpsTagArray__tags  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_members = {
  "kalman_interfaces__msg",  // message namespace
  "GpsTagArray",  // message name
  1,  // number of fields
  sizeof(kalman_interfaces__msg__GpsTagArray),
  kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_member_array,  // message members
  kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_init_function,  // function to initialize message memory (memory has to be allocated)
  kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_type_support_handle = {
  0,
  &kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kalman_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, msg, GpsTagArray)() {
  kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kalman_interfaces, msg, GpsTag)();
  if (!kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_type_support_handle.typesupport_identifier) {
    kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kalman_interfaces__msg__GpsTagArray__rosidl_typesupport_introspection_c__GpsTagArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
