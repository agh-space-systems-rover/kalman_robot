// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
// generated code does not contain a copyright notice

#ifndef UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__STRUCT_H_
#define UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CameraMetadata in the package unity_rs_publisher_msgs.
typedef struct unity_rs_publisher_msgs__msg__CameraMetadata
{
  uint32_t width;
  uint32_t height;
  float depth_min;
  float depth_max;
  float fx;
  float fy;
  float cx;
  float cy;
} unity_rs_publisher_msgs__msg__CameraMetadata;

// Struct for a sequence of unity_rs_publisher_msgs__msg__CameraMetadata.
typedef struct unity_rs_publisher_msgs__msg__CameraMetadata__Sequence
{
  unity_rs_publisher_msgs__msg__CameraMetadata * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unity_rs_publisher_msgs__msg__CameraMetadata__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__STRUCT_H_
