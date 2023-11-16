// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__FUNCTIONS_H_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kalman_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "kalman_interfaces/msg/detail/gps_tag_array__struct.h"

/// Initialize msg/GpsTagArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kalman_interfaces__msg__GpsTagArray
 * )) before or use
 * kalman_interfaces__msg__GpsTagArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTagArray__init(kalman_interfaces__msg__GpsTagArray * msg);

/// Finalize msg/GpsTagArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTagArray__fini(kalman_interfaces__msg__GpsTagArray * msg);

/// Create msg/GpsTagArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kalman_interfaces__msg__GpsTagArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
kalman_interfaces__msg__GpsTagArray *
kalman_interfaces__msg__GpsTagArray__create();

/// Destroy msg/GpsTagArray message.
/**
 * It calls
 * kalman_interfaces__msg__GpsTagArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTagArray__destroy(kalman_interfaces__msg__GpsTagArray * msg);

/// Check for msg/GpsTagArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTagArray__are_equal(const kalman_interfaces__msg__GpsTagArray * lhs, const kalman_interfaces__msg__GpsTagArray * rhs);

/// Copy a msg/GpsTagArray message.
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
kalman_interfaces__msg__GpsTagArray__copy(
  const kalman_interfaces__msg__GpsTagArray * input,
  kalman_interfaces__msg__GpsTagArray * output);

/// Initialize array of msg/GpsTagArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * kalman_interfaces__msg__GpsTagArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTagArray__Sequence__init(kalman_interfaces__msg__GpsTagArray__Sequence * array, size_t size);

/// Finalize array of msg/GpsTagArray messages.
/**
 * It calls
 * kalman_interfaces__msg__GpsTagArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTagArray__Sequence__fini(kalman_interfaces__msg__GpsTagArray__Sequence * array);

/// Create array of msg/GpsTagArray messages.
/**
 * It allocates the memory for the array and calls
 * kalman_interfaces__msg__GpsTagArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
kalman_interfaces__msg__GpsTagArray__Sequence *
kalman_interfaces__msg__GpsTagArray__Sequence__create(size_t size);

/// Destroy array of msg/GpsTagArray messages.
/**
 * It calls
 * kalman_interfaces__msg__GpsTagArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
void
kalman_interfaces__msg__GpsTagArray__Sequence__destroy(kalman_interfaces__msg__GpsTagArray__Sequence * array);

/// Check for msg/GpsTagArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kalman_interfaces
bool
kalman_interfaces__msg__GpsTagArray__Sequence__are_equal(const kalman_interfaces__msg__GpsTagArray__Sequence * lhs, const kalman_interfaces__msg__GpsTagArray__Sequence * rhs);

/// Copy an array of msg/GpsTagArray messages.
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
kalman_interfaces__msg__GpsTagArray__Sequence__copy(
  const kalman_interfaces__msg__GpsTagArray__Sequence * input,
  kalman_interfaces__msg__GpsTagArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__FUNCTIONS_H_
