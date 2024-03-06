// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:srv/SetUeuosColor.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__STRUCT_H_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__struct.h"

/// Struct defined in srv/SetUeuosColor in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__SetUeuosColor_Request
{
  /// Alpha is unused.
  std_msgs__msg__ColorRGBA color;
} kalman_interfaces__srv__SetUeuosColor_Request;

// Struct for a sequence of kalman_interfaces__srv__SetUeuosColor_Request.
typedef struct kalman_interfaces__srv__SetUeuosColor_Request__Sequence
{
  kalman_interfaces__srv__SetUeuosColor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__SetUeuosColor_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetUeuosColor in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__SetUeuosColor_Response
{
  uint8_t structure_needs_at_least_one_member;
} kalman_interfaces__srv__SetUeuosColor_Response;

// Struct for a sequence of kalman_interfaces__srv__SetUeuosColor_Response.
typedef struct kalman_interfaces__srv__SetUeuosColor_Response__Sequence
{
  kalman_interfaces__srv__SetUeuosColor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__SetUeuosColor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__STRUCT_H_
