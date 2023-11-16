// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:msg/GpsTag.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__STRUCT_H_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GpsTag in the package kalman_interfaces.
typedef struct kalman_interfaces__msg__GpsTag
{
  int8_t tag_id;
  float tag_lat;
  float tag_lon;
} kalman_interfaces__msg__GpsTag;

// Struct for a sequence of kalman_interfaces__msg__GpsTag.
typedef struct kalman_interfaces__msg__GpsTag__Sequence
{
  kalman_interfaces__msg__GpsTag * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__msg__GpsTag__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__STRUCT_H_
