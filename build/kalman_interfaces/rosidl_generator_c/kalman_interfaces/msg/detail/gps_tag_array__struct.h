// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__STRUCT_H_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tags'
#include "kalman_interfaces/msg/detail/gps_tag__struct.h"

/// Struct defined in msg/GpsTagArray in the package kalman_interfaces.
typedef struct kalman_interfaces__msg__GpsTagArray
{
  kalman_interfaces__msg__GpsTag__Sequence tags;
} kalman_interfaces__msg__GpsTagArray;

// Struct for a sequence of kalman_interfaces__msg__GpsTagArray.
typedef struct kalman_interfaces__msg__GpsTagArray__Sequence
{
  kalman_interfaces__msg__GpsTagArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__msg__GpsTagArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__STRUCT_H_
