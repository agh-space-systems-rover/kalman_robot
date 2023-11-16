// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/msg/detail/gps_tag_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tags`
#include "kalman_interfaces/msg/detail/gps_tag__functions.h"

bool
kalman_interfaces__msg__GpsTagArray__init(kalman_interfaces__msg__GpsTagArray * msg)
{
  if (!msg) {
    return false;
  }
  // tags
  if (!kalman_interfaces__msg__GpsTag__Sequence__init(&msg->tags, 0)) {
    kalman_interfaces__msg__GpsTagArray__fini(msg);
    return false;
  }
  return true;
}

void
kalman_interfaces__msg__GpsTagArray__fini(kalman_interfaces__msg__GpsTagArray * msg)
{
  if (!msg) {
    return;
  }
  // tags
  kalman_interfaces__msg__GpsTag__Sequence__fini(&msg->tags);
}

bool
kalman_interfaces__msg__GpsTagArray__are_equal(const kalman_interfaces__msg__GpsTagArray * lhs, const kalman_interfaces__msg__GpsTagArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tags
  if (!kalman_interfaces__msg__GpsTag__Sequence__are_equal(
      &(lhs->tags), &(rhs->tags)))
  {
    return false;
  }
  return true;
}

bool
kalman_interfaces__msg__GpsTagArray__copy(
  const kalman_interfaces__msg__GpsTagArray * input,
  kalman_interfaces__msg__GpsTagArray * output)
{
  if (!input || !output) {
    return false;
  }
  // tags
  if (!kalman_interfaces__msg__GpsTag__Sequence__copy(
      &(input->tags), &(output->tags)))
  {
    return false;
  }
  return true;
}

kalman_interfaces__msg__GpsTagArray *
kalman_interfaces__msg__GpsTagArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__GpsTagArray * msg = (kalman_interfaces__msg__GpsTagArray *)allocator.allocate(sizeof(kalman_interfaces__msg__GpsTagArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kalman_interfaces__msg__GpsTagArray));
  bool success = kalman_interfaces__msg__GpsTagArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kalman_interfaces__msg__GpsTagArray__destroy(kalman_interfaces__msg__GpsTagArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kalman_interfaces__msg__GpsTagArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kalman_interfaces__msg__GpsTagArray__Sequence__init(kalman_interfaces__msg__GpsTagArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__GpsTagArray * data = NULL;

  if (size) {
    data = (kalman_interfaces__msg__GpsTagArray *)allocator.zero_allocate(size, sizeof(kalman_interfaces__msg__GpsTagArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kalman_interfaces__msg__GpsTagArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kalman_interfaces__msg__GpsTagArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
kalman_interfaces__msg__GpsTagArray__Sequence__fini(kalman_interfaces__msg__GpsTagArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      kalman_interfaces__msg__GpsTagArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

kalman_interfaces__msg__GpsTagArray__Sequence *
kalman_interfaces__msg__GpsTagArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__GpsTagArray__Sequence * array = (kalman_interfaces__msg__GpsTagArray__Sequence *)allocator.allocate(sizeof(kalman_interfaces__msg__GpsTagArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kalman_interfaces__msg__GpsTagArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kalman_interfaces__msg__GpsTagArray__Sequence__destroy(kalman_interfaces__msg__GpsTagArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kalman_interfaces__msg__GpsTagArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kalman_interfaces__msg__GpsTagArray__Sequence__are_equal(const kalman_interfaces__msg__GpsTagArray__Sequence * lhs, const kalman_interfaces__msg__GpsTagArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kalman_interfaces__msg__GpsTagArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kalman_interfaces__msg__GpsTagArray__Sequence__copy(
  const kalman_interfaces__msg__GpsTagArray__Sequence * input,
  kalman_interfaces__msg__GpsTagArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kalman_interfaces__msg__GpsTagArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kalman_interfaces__msg__GpsTagArray * data =
      (kalman_interfaces__msg__GpsTagArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kalman_interfaces__msg__GpsTagArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kalman_interfaces__msg__GpsTagArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kalman_interfaces__msg__GpsTagArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
