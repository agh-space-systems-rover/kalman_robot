// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/msg/detail/wheel_states__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "kalman_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "kalman_interfaces/msg/detail/wheel_states__struct.h"
#include "kalman_interfaces/msg/detail/wheel_states__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "kalman_interfaces/msg/detail/wheel_state__functions.h"  // back_left, back_right, front_left, front_right

// forward declare type support functions
size_t get_serialized_size_kalman_interfaces__msg__WheelState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_kalman_interfaces__msg__WheelState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState)();


using _WheelStates__ros_msg_type = kalman_interfaces__msg__WheelStates;

static bool _WheelStates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WheelStates__ros_msg_type * ros_message = static_cast<const _WheelStates__ros_msg_type *>(untyped_ros_message);
  // Field name: front_left
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->front_left, cdr))
    {
      return false;
    }
  }

  // Field name: front_right
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->front_right, cdr))
    {
      return false;
    }
  }

  // Field name: back_left
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->back_left, cdr))
    {
      return false;
    }
  }

  // Field name: back_right
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->back_right, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _WheelStates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WheelStates__ros_msg_type * ros_message = static_cast<_WheelStates__ros_msg_type *>(untyped_ros_message);
  // Field name: front_left
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->front_left))
    {
      return false;
    }
  }

  // Field name: front_right
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->front_right))
    {
      return false;
    }
  }

  // Field name: back_left
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->back_left))
    {
      return false;
    }
  }

  // Field name: back_right
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->back_right))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kalman_interfaces
size_t get_serialized_size_kalman_interfaces__msg__WheelStates(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WheelStates__ros_msg_type * ros_message = static_cast<const _WheelStates__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name front_left

  current_alignment += get_serialized_size_kalman_interfaces__msg__WheelState(
    &(ros_message->front_left), current_alignment);
  // field.name front_right

  current_alignment += get_serialized_size_kalman_interfaces__msg__WheelState(
    &(ros_message->front_right), current_alignment);
  // field.name back_left

  current_alignment += get_serialized_size_kalman_interfaces__msg__WheelState(
    &(ros_message->back_left), current_alignment);
  // field.name back_right

  current_alignment += get_serialized_size_kalman_interfaces__msg__WheelState(
    &(ros_message->back_right), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _WheelStates__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_kalman_interfaces__msg__WheelStates(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kalman_interfaces
size_t max_serialized_size_kalman_interfaces__msg__WheelStates(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: front_left
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_kalman_interfaces__msg__WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: front_right
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_kalman_interfaces__msg__WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: back_left
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_kalman_interfaces__msg__WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: back_right
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_kalman_interfaces__msg__WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = kalman_interfaces__msg__WheelStates;
    is_plain =
      (
      offsetof(DataType, back_right) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _WheelStates__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_kalman_interfaces__msg__WheelStates(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_WheelStates = {
  "kalman_interfaces::msg",
  "WheelStates",
  _WheelStates__cdr_serialize,
  _WheelStates__cdr_deserialize,
  _WheelStates__get_serialized_size,
  _WheelStates__max_serialized_size
};

static rosidl_message_type_support_t _WheelStates__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WheelStates,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, msg, WheelStates)() {
  return &_WheelStates__type_support;
}

#if defined(__cplusplus)
}
#endif
