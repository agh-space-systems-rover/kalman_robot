// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__STRUCT_H_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'front_left'
// Member 'front_right'
// Member 'back_left'
// Member 'back_right'
#include "kalman_interfaces/msg/detail/wheel_state__struct.h"

/// Struct defined in msg/WheelStates in the package kalman_interfaces.
typedef struct kalman_interfaces__msg__WheelStates
{
  kalman_interfaces__msg__WheelState front_left;
  kalman_interfaces__msg__WheelState front_right;
  kalman_interfaces__msg__WheelState back_left;
  kalman_interfaces__msg__WheelState back_right;
} kalman_interfaces__msg__WheelStates;

// Struct for a sequence of kalman_interfaces__msg__WheelStates.
typedef struct kalman_interfaces__msg__WheelStates__Sequence
{
  kalman_interfaces__msg__WheelStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__msg__WheelStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__STRUCT_H_
