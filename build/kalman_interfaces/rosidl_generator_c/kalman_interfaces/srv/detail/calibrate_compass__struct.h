// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__STRUCT_H_
#define KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CalibrateCompass in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__CalibrateCompass_Request
{
  float duration;
  float angular_velocity;
} kalman_interfaces__srv__CalibrateCompass_Request;

// Struct for a sequence of kalman_interfaces__srv__CalibrateCompass_Request.
typedef struct kalman_interfaces__srv__CalibrateCompass_Request__Sequence
{
  kalman_interfaces__srv__CalibrateCompass_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__CalibrateCompass_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CalibrateCompass in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__CalibrateCompass_Response
{
  bool success;
  float cc_mag_field;
  float cc_offset0;
  float cc_offset1;
  float cc_offset2;
  float cc_gain0;
  float cc_gain1;
  float cc_gain2;
  float cc_t0;
  float cc_t1;
  float cc_t2;
  float cc_t3;
  float cc_t4;
  float cc_t5;
} kalman_interfaces__srv__CalibrateCompass_Response;

// Struct for a sequence of kalman_interfaces__srv__CalibrateCompass_Response.
typedef struct kalman_interfaces__srv__CalibrateCompass_Response__Sequence
{
  kalman_interfaces__srv__CalibrateCompass_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__CalibrateCompass_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__STRUCT_H_
