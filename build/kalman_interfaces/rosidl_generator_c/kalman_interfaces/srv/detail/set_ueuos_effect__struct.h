// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kalman_interfaces:srv/SetUeuosEffect.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_EFFECT__STRUCT_H_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_EFFECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'BOOT'.
enum
{
  kalman_interfaces__srv__SetUeuosEffect_Request__BOOT = 0
};

/// Constant 'RAINBOW'.
enum
{
  kalman_interfaces__srv__SetUeuosEffect_Request__RAINBOW = 1
};

/// Struct defined in srv/SetUeuosEffect in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__SetUeuosEffect_Request
{
  uint8_t effect;
} kalman_interfaces__srv__SetUeuosEffect_Request;

// Struct for a sequence of kalman_interfaces__srv__SetUeuosEffect_Request.
typedef struct kalman_interfaces__srv__SetUeuosEffect_Request__Sequence
{
  kalman_interfaces__srv__SetUeuosEffect_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__SetUeuosEffect_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetUeuosEffect in the package kalman_interfaces.
typedef struct kalman_interfaces__srv__SetUeuosEffect_Response
{
  uint8_t structure_needs_at_least_one_member;
} kalman_interfaces__srv__SetUeuosEffect_Response;

// Struct for a sequence of kalman_interfaces__srv__SetUeuosEffect_Response.
typedef struct kalman_interfaces__srv__SetUeuosEffect_Response__Sequence
{
  kalman_interfaces__srv__SetUeuosEffect_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kalman_interfaces__srv__SetUeuosEffect_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_EFFECT__STRUCT_H_
