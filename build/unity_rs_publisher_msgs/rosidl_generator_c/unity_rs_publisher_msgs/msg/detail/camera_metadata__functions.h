// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
// generated code does not contain a copyright notice

#ifndef UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__FUNCTIONS_H_
#define UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "unity_rs_publisher_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "unity_rs_publisher_msgs/msg/detail/camera_metadata__struct.h"

/// Initialize msg/CameraMetadata message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * unity_rs_publisher_msgs__msg__CameraMetadata
 * )) before or use
 * unity_rs_publisher_msgs__msg__CameraMetadata__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
bool
unity_rs_publisher_msgs__msg__CameraMetadata__init(unity_rs_publisher_msgs__msg__CameraMetadata * msg);

/// Finalize msg/CameraMetadata message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
void
unity_rs_publisher_msgs__msg__CameraMetadata__fini(unity_rs_publisher_msgs__msg__CameraMetadata * msg);

/// Create msg/CameraMetadata message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * unity_rs_publisher_msgs__msg__CameraMetadata__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
unity_rs_publisher_msgs__msg__CameraMetadata *
unity_rs_publisher_msgs__msg__CameraMetadata__create();

/// Destroy msg/CameraMetadata message.
/**
 * It calls
 * unity_rs_publisher_msgs__msg__CameraMetadata__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
void
unity_rs_publisher_msgs__msg__CameraMetadata__destroy(unity_rs_publisher_msgs__msg__CameraMetadata * msg);

/// Check for msg/CameraMetadata message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
bool
unity_rs_publisher_msgs__msg__CameraMetadata__are_equal(const unity_rs_publisher_msgs__msg__CameraMetadata * lhs, const unity_rs_publisher_msgs__msg__CameraMetadata * rhs);

/// Copy a msg/CameraMetadata message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
bool
unity_rs_publisher_msgs__msg__CameraMetadata__copy(
  const unity_rs_publisher_msgs__msg__CameraMetadata * input,
  unity_rs_publisher_msgs__msg__CameraMetadata * output);

/// Initialize array of msg/CameraMetadata messages.
/**
 * It allocates the memory for the number of elements and calls
 * unity_rs_publisher_msgs__msg__CameraMetadata__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
bool
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__init(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array, size_t size);

/// Finalize array of msg/CameraMetadata messages.
/**
 * It calls
 * unity_rs_publisher_msgs__msg__CameraMetadata__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
void
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__fini(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array);

/// Create array of msg/CameraMetadata messages.
/**
 * It allocates the memory for the array and calls
 * unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence *
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__create(size_t size);

/// Destroy array of msg/CameraMetadata messages.
/**
 * It calls
 * unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
void
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__destroy(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array);

/// Check for msg/CameraMetadata message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
bool
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__are_equal(const unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * lhs, const unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * rhs);

/// Copy an array of msg/CameraMetadata messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_unity_rs_publisher_msgs
bool
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__copy(
  const unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * input,
  unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__FUNCTIONS_H_
