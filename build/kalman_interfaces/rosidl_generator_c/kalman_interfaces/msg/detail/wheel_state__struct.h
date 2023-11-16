// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:msg/WheelState.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__STRUCT_H_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WheelState in the package kalman_interfaces.
/**
  * linear velocity in m/s
 */
typedef struct kalman_interfaces__msg__WheelState
{
  float velocity;
  /// radians; 0 is forward; pi/2 is left; -pi/2 is right
  float angle;
} kalman_interfaces__msg__WheelState;

// Struct for a sequence of kalman_interfaces__msg__WheelState.
typedef struct kalman_interfaces__msg__WheelState__Sequence
{
  kalman_interfaces__msg__WheelState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__msg__WheelState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__STRUCT_H_
