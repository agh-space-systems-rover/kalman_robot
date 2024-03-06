// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:srv/SetUeuosMode.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__STRUCT_H_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OFF'.
enum
{
  kalman_interfaces__srv__SetUeuosMode_Request__OFF = 0
};

/// Constant 'AUTONOMY'.
enum
{
  kalman_interfaces__srv__SetUeuosMode_Request__AUTONOMY = 1
};

/// Constant 'TELEOP'.
enum
{
  kalman_interfaces__srv__SetUeuosMode_Request__TELEOP = 2
};

/// Constant 'FINISHED'.
enum
{
  kalman_interfaces__srv__SetUeuosMode_Request__FINISHED = 3
};

/// Struct defined in srv/SetUeuosMode in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__SetUeuosMode_Request
{
  uint8_t mode;
} kalman_interfaces__srv__SetUeuosMode_Request;

// Struct for a sequence of kalman_interfaces__srv__SetUeuosMode_Request.
typedef struct kalman_interfaces__srv__SetUeuosMode_Request__Sequence
{
  kalman_interfaces__srv__SetUeuosMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__SetUeuosMode_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetUeuosMode in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__SetUeuosMode_Response
{
  uint8_t structure_needs_at_least_one_member;
} kalman_interfaces__srv__SetUeuosMode_Response;

// Struct for a sequence of kalman_interfaces__srv__SetUeuosMode_Response.
typedef struct kalman_interfaces__srv__SetUeuosMode_Response__Sequence
{
  kalman_interfaces__srv__SetUeuosMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__SetUeuosMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__STRUCT_H_
