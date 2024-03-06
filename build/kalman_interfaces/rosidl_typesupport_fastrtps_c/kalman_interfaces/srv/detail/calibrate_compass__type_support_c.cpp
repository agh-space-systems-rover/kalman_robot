// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/srv/detail/calibrate_compass__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "kalman_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "kalman_interfaces/srv/detail/calibrate_compass__struct.h"
#include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"
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


// forward declare type support functions


using _CalibrateCompass_Request__ros_msg_type = kalman_interfaces__srv__CalibrateCompass_Request;

static bool _CalibrateCompass_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CalibrateCompass_Request__ros_msg_type * ros_message = static_cast<const _CalibrateCompass_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: duration
  {
    cdr << ros_message->duration;
  }

  // Field name: angular_velocity
  {
    cdr << ros_message->angular_velocity;
  }

  return true;
}

static bool _CalibrateCompass_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CalibrateCompass_Request__ros_msg_type * ros_message = static_cast<_CalibrateCompass_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: duration
  {
    cdr >> ros_message->duration;
  }

  // Field name: angular_velocity
  {
    cdr >> ros_message->angular_velocity;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kalman_interfaces
size_t get_serialized_size_kalman_interfaces__srv__CalibrateCompass_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CalibrateCompass_Request__ros_msg_type * ros_message = static_cast<const _CalibrateCompass_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name duration
  {
    size_t item_size = sizeof(ros_message->duration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angular_velocity
  {
    size_t item_size = sizeof(ros_message->angular_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CalibrateCompass_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_kalman_interfaces__srv__CalibrateCompass_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kalman_interfaces
size_t max_serialized_size_kalman_interfaces__srv__CalibrateCompass_Request(
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

  // member: duration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: angular_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = kalman_interfaces__srv__CalibrateCompass_Request;
    is_plain =
      (
      offsetof(DataType, angular_velocity) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CalibrateCompass_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_kalman_interfaces__srv__CalibrateCompass_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CalibrateCompass_Request = {
  "kalman_interfaces::srv",
  "CalibrateCompass_Request",
  _CalibrateCompass_Request__cdr_serialize,
  _CalibrateCompass_Request__cdr_deserialize,
  _CalibrateCompass_Request__get_serialized_size,
  _CalibrateCompass_Request__max_serialized_size
};

static rosidl_message_type_support_t _CalibrateCompass_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CalibrateCompass_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, srv, CalibrateCompass_Request)() {
  return &_CalibrateCompass_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "kalman_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__struct.h"
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


// forward declare type support functions


using _CalibrateCompass_Response__ros_msg_type = kalman_interfaces__srv__CalibrateCompass_Response;

static bool _CalibrateCompass_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CalibrateCompass_Response__ros_msg_type * ros_message = static_cast<const _CalibrateCompass_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: cc_mag_field
  {
    cdr << ros_message->cc_mag_field;
  }

  // Field name: cc_offset0
  {
    cdr << ros_message->cc_offset0;
  }

  // Field name: cc_offset1
  {
    cdr << ros_message->cc_offset1;
  }

  // Field name: cc_offset2
  {
    cdr << ros_message->cc_offset2;
  }

  // Field name: cc_gain0
  {
    cdr << ros_message->cc_gain0;
  }

  // Field name: cc_gain1
  {
    cdr << ros_message->cc_gain1;
  }

  // Field name: cc_gain2
  {
    cdr << ros_message->cc_gain2;
  }

  // Field name: cc_t0
  {
    cdr << ros_message->cc_t0;
  }

  // Field name: cc_t1
  {
    cdr << ros_message->cc_t1;
  }

  // Field name: cc_t2
  {
    cdr << ros_message->cc_t2;
  }

  // Field name: cc_t3
  {
    cdr << ros_message->cc_t3;
  }

  // Field name: cc_t4
  {
    cdr << ros_message->cc_t4;
  }

  // Field name: cc_t5
  {
    cdr << ros_message->cc_t5;
  }

  return true;
}

static bool _CalibrateCompass_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CalibrateCompass_Response__ros_msg_type * ros_message = static_cast<_CalibrateCompass_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: cc_mag_field
  {
    cdr >> ros_message->cc_mag_field;
  }

  // Field name: cc_offset0
  {
    cdr >> ros_message->cc_offset0;
  }

  // Field name: cc_offset1
  {
    cdr >> ros_message->cc_offset1;
  }

  // Field name: cc_offset2
  {
    cdr >> ros_message->cc_offset2;
  }

  // Field name: cc_gain0
  {
    cdr >> ros_message->cc_gain0;
  }

  // Field name: cc_gain1
  {
    cdr >> ros_message->cc_gain1;
  }

  // Field name: cc_gain2
  {
    cdr >> ros_message->cc_gain2;
  }

  // Field name: cc_t0
  {
    cdr >> ros_message->cc_t0;
  }

  // Field name: cc_t1
  {
    cdr >> ros_message->cc_t1;
  }

  // Field name: cc_t2
  {
    cdr >> ros_message->cc_t2;
  }

  // Field name: cc_t3
  {
    cdr >> ros_message->cc_t3;
  }

  // Field name: cc_t4
  {
    cdr >> ros_message->cc_t4;
  }

  // Field name: cc_t5
  {
    cdr >> ros_message->cc_t5;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kalman_interfaces
size_t get_serialized_size_kalman_interfaces__srv__CalibrateCompass_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CalibrateCompass_Response__ros_msg_type * ros_message = static_cast<const _CalibrateCompass_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_mag_field
  {
    size_t item_size = sizeof(ros_message->cc_mag_field);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_offset0
  {
    size_t item_size = sizeof(ros_message->cc_offset0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_offset1
  {
    size_t item_size = sizeof(ros_message->cc_offset1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_offset2
  {
    size_t item_size = sizeof(ros_message->cc_offset2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_gain0
  {
    size_t item_size = sizeof(ros_message->cc_gain0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_gain1
  {
    size_t item_size = sizeof(ros_message->cc_gain1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_gain2
  {
    size_t item_size = sizeof(ros_message->cc_gain2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_t0
  {
    size_t item_size = sizeof(ros_message->cc_t0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_t1
  {
    size_t item_size = sizeof(ros_message->cc_t1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_t2
  {
    size_t item_size = sizeof(ros_message->cc_t2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_t3
  {
    size_t item_size = sizeof(ros_message->cc_t3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_t4
  {
    size_t item_size = sizeof(ros_message->cc_t4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cc_t5
  {
    size_t item_size = sizeof(ros_message->cc_t5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CalibrateCompass_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_kalman_interfaces__srv__CalibrateCompass_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kalman_interfaces
size_t max_serialized_size_kalman_interfaces__srv__CalibrateCompass_Response(
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

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cc_mag_field
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_offset0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_offset1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_offset2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_gain0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_gain1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_gain2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_t0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_t1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_t2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_t3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_t4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cc_t5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = kalman_interfaces__srv__CalibrateCompass_Response;
    is_plain =
      (
      offsetof(DataType, cc_t5) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CalibrateCompass_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_kalman_interfaces__srv__CalibrateCompass_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CalibrateCompass_Response = {
  "kalman_interfaces::srv",
  "CalibrateCompass_Response",
  _CalibrateCompass_Response__cdr_serialize,
  _CalibrateCompass_Response__cdr_deserialize,
  _CalibrateCompass_Response__get_serialized_size,
  _CalibrateCompass_Response__max_serialized_size
};

static rosidl_message_type_support_t _CalibrateCompass_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CalibrateCompass_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, srv, CalibrateCompass_Response)() {
  return &_CalibrateCompass_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "kalman_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "kalman_interfaces/srv/calibrate_compass.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t CalibrateCompass__callbacks = {
  "kalman_interfaces::srv",
  "CalibrateCompass",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, srv, CalibrateCompass_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, srv, CalibrateCompass_Response)(),
};

static rosidl_service_type_support_t CalibrateCompass__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &CalibrateCompass__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kalman_interfaces, srv, CalibrateCompass)() {
  return &CalibrateCompass__handle;
}

#if defined(__cplusplus)
}
#endif
