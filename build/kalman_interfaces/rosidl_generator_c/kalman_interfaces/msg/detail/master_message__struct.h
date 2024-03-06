// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:msg/MasterMessage.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__STRUCT_H_
#define KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UEUOS_SET_STATE'.
enum
{
  kalman_interfaces__msg__MasterMessage__UEUOS_SET_STATE = 96
};

/// Constant 'UEUOS_SET_COLOR'.
enum
{
  kalman_interfaces__msg__MasterMessage__UEUOS_SET_COLOR = 97
};

/// Constant 'UEUOS_SET_EFFECT'.
enum
{
  kalman_interfaces__msg__MasterMessage__UEUOS_SET_EFFECT = 98
};

/// Constant 'MOTOR_SET_WHEELS'.
enum
{
  kalman_interfaces__msg__MasterMessage__MOTOR_SET_WHEELS = 64
};

/// Constant 'AUTONOMY_SWITCH'.
enum
{
  kalman_interfaces__msg__MasterMessage__AUTONOMY_SWITCH = 32
};

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MasterMessage in the package kalman_interfaces.
typedef struct kalman_interfaces__msg__MasterMessage
{
  /// command number
  uint8_t cmd;
  /// command data
  rosidl_runtime_c__uint8__Sequence data;
} kalman_interfaces__msg__MasterMessage;

// Struct for a sequence of kalman_interfaces__msg__MasterMessage.
typedef struct kalman_interfaces__msg__MasterMessage__Sequence
{
  kalman_interfaces__msg__MasterMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__msg__MasterMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__STRUCT_H_
