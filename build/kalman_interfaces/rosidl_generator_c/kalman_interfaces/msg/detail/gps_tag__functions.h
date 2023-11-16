// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kalman_interfaces:msg/GpsTag.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__FUNCTIONS_H_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kalman_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "kalman_interfaces/msg/detail/gps_tag__struct.h"

/// Initialize msg/GpsTag message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kalman_interfaces__msg__GpsTag
 * )) before or use
 * kalman_interfaces__msg__GpsTag__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTag__init(kalman_interfaces__msg__GpsTag * msg);

/// Finalize msg/GpsTag message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTag__fini(kalman_interfaces__msg__GpsTag * msg);

/// Create msg/GpsTag message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kalman_interfaces__msg__GpsTag__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
kalman_interfaces__msg__GpsTag *
kalman_interfaces__msg__GpsTag__create();

/// Destroy msg/GpsTag message.
/**
 * It calls
 * kalman_interfaces__msg__GpsTag__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTag__destroy(kalman_interfaces__msg__GpsTag * msg);

/// Check for msg/GpsTag message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTag__are_equal(const kalman_interfaces__msg__GpsTag * lhs, const kalman_interfaces__msg__GpsTag * rhs);

/// Copy a msg/GpsTag message.
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
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTag__copy(
  const kalman_interfaces__msg__GpsTag * input,
  kalman_interfaces__msg__GpsTag * output);

/// Initialize array of msg/GpsTag messages.
/**
 * It allocates the memory for the number of elements and calls
 * kalman_interfaces__msg__GpsTag__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTag__Sequence__init(kalman_interfaces__msg__GpsTag__Sequence * array, size_t size);

/// Finalize array of msg/GpsTag messages.
/**
 * It calls
 * kalman_interfaces__msg__GpsTag__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTag__Sequence__fini(kalman_interfaces__msg__GpsTag__Sequence * array);

/// Create array of msg/GpsTag messages.
/**
 * It allocates the memory for the array and calls
 * kalman_interfaces__msg__GpsTag__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
kalman_interfaces__msg__GpsTag__Sequence *
kalman_interfaces__msg__GpsTag__Sequence__create(size_t size);

/// Destroy array of msg/GpsTag messages.
/**
 * It calls
 * kalman_interfaces__msg__GpsTag__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTag__Sequence__destroy(kalman_interfaces__msg__GpsTag__Sequence * array);

/// Check for msg/GpsTag message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTag__Sequence__are_equal(const kalman_interfaces__msg__GpsTag__Sequence * lhs, const kalman_interfaces__msg__GpsTag__Sequence * rhs);

/// Copy an array of msg/GpsTag messages.
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
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTag__Sequence__copy(
  const kalman_interfaces__msg__GpsTag__Sequence * input,
  kalman_interfaces__msg__GpsTag__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__FUNCTIONS_H_
