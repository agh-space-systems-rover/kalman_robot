// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/msg/detail/wheel_states__rosidl_typesupport_fastrtps_cpp.hpp"
#include "kalman_interfaces/msg/detail/wheel_states__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace kalman_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const kalman_interfaces::msg::WheelState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  kalman_interfaces::msg::WheelState &);
size_t get_serialized_size(
  const kalman_interfaces::msg::WheelState &,
  size_t current_alignment);
size_t
max_serialized_size_WheelState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace kalman_interfaces

// functions for kalman_interfaces::msg::WheelState already declared above

// functions for kalman_interfaces::msg::WheelState already declared above

// functions for kalman_interfaces::msg::WheelState already declared above


namespace kalman_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kalman_interfaces
cdr_serialize(
  const kalman_interfaces::msg::WheelStates & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: front_left
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.front_left,
    cdr);
  // Member: front_right
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.front_right,
    cdr);
  // Member: back_left
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.back_left,
    cdr);
  // Member: back_right
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.back_right,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kalman_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  kalman_interfaces::msg::WheelStates & ros_message)
{
  // Member: front_left
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.front_left);

  // Member: front_right
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.front_right);

  // Member: back_left
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.back_left);

  // Member: back_right
  kalman_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.back_right);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kalman_interfaces
get_serialized_size(
  const kalman_interfaces::msg::WheelStates & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: front_left

  current_alignment +=
    kalman_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.front_left, current_alignment);
  // Member: front_right

  current_alignment +=
    kalman_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.front_right, current_alignment);
  // Member: back_left

  current_alignment +=
    kalman_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.back_left, current_alignment);
  // Member: back_right

  current_alignment +=
    kalman_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.back_right, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kalman_interfaces
max_serialized_size_WheelStates(
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


  // Member: front_left
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        kalman_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: front_right
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        kalman_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: back_left
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        kalman_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_WheelState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: back_right
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        kalman_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_WheelState(
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
    using DataType = kalman_interfaces::msg::WheelStates;
    is_plain =
      (
      offsetof(DataType, back_right) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _WheelStates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const kalman_interfaces::msg::WheelStates *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _WheelStates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<kalman_interfaces::msg::WheelStates *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _WheelStates__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const kalman_interfaces::msg::WheelStates *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _WheelStates__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_WheelStates(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _WheelStates__callbacks = {
  "kalman_interfaces::msg",
  "WheelStates",
  _WheelStates__cdr_serialize,
  _WheelStates__cdr_deserialize,
  _WheelStates__get_serialized_size,
  _WheelStates__max_serialized_size
};

static rosidl_message_type_support_t _WheelStates__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_WheelStates__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace kalman_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_kalman_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<kalman_interfaces::msg::WheelStates>()
{
  return &kalman_interfaces::msg::typesupport_fastrtps_cpp::_WheelStates__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kalman_interfaces, msg, WheelStates)() {
  return &kalman_interfaces::msg::typesupport_fastrtps_cpp::_WheelStates__handle;
}

#ifdef __cplusplus
}
#endif
