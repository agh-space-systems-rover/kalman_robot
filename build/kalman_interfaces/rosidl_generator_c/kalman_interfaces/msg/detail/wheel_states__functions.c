// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/msg/detail/wheel_states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `front_left`
// Member `front_right`
// Member `back_left`
// Member `back_right`
#include "kalman_interfaces/msg/detail/wheel_state__functions.h"

bool
kalman_interfaces__msg__WheelStates__init(kalman_interfaces__msg__WheelStates * msg)
{
  if (!msg) {
    return false;
  }
  // front_left
  if (!kalman_interfaces__msg__WheelState__init(&msg->front_left)) {
    kalman_interfaces__msg__WheelStates__fini(msg);
    return false;
  }
  // front_right
  if (!kalman_interfaces__msg__WheelState__init(&msg->front_right)) {
    kalman_interfaces__msg__WheelStates__fini(msg);
    return false;
  }
  // back_left
  if (!kalman_interfaces__msg__WheelState__init(&msg->back_left)) {
    kalman_interfaces__msg__WheelStates__fini(msg);
    return false;
  }
  // back_right
  if (!kalman_interfaces__msg__WheelState__init(&msg->back_right)) {
    kalman_interfaces__msg__WheelStates__fini(msg);
    return false;
  }
  return true;
}

void
kalman_interfaces__msg__WheelStates__fini(kalman_interfaces__msg__WheelStates * msg)
{
  if (!msg) {
    return;
  }
  // front_left
  kalman_interfaces__msg__WheelState__fini(&msg->front_left);
  // front_right
  kalman_interfaces__msg__WheelState__fini(&msg->front_right);
  // back_left
  kalman_interfaces__msg__WheelState__fini(&msg->back_left);
  // back_right
  kalman_interfaces__msg__WheelState__fini(&msg->back_right);
}

bool
kalman_interfaces__msg__WheelStates__are_equal(const kalman_interfaces__msg__WheelStates * lhs, const kalman_interfaces__msg__WheelStates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front_left
  if (!kalman_interfaces__msg__WheelState__are_equal(
      &(lhs->front_left), &(rhs->front_left)))
  {
    return false;
  }
  // front_right
  if (!kalman_interfaces__msg__WheelState__are_equal(
      &(lhs->front_right), &(rhs->front_right)))
  {
    return false;
  }
  // back_left
  if (!kalman_interfaces__msg__WheelState__are_equal(
      &(lhs->back_left), &(rhs->back_left)))
  {
    return false;
  }
  // back_right
  if (!kalman_interfaces__msg__WheelState__are_equal(
      &(lhs->back_right), &(rhs->back_right)))
  {
    return false;
  }
  return true;
}

bool
kalman_interfaces__msg__WheelStates__copy(
  const kalman_interfaces__msg__WheelStates * input,
  kalman_interfaces__msg__WheelStates * output)
{
  if (!input || !output) {
    return false;
  }
  // front_left
  if (!kalman_interfaces__msg__WheelState__copy(
      &(input->front_left), &(output->front_left)))
  {
    return false;
  }
  // front_right
  if (!kalman_interfaces__msg__WheelState__copy(
      &(input->front_right), &(output->front_right)))
  {
    return false;
  }
  // back_left
  if (!kalman_interfaces__msg__WheelState__copy(
      &(input->back_left), &(output->back_left)))
  {
    return false;
  }
  // back_right
  if (!kalman_interfaces__msg__WheelState__copy(
      &(input->back_right), &(output->back_right)))
  {
    return false;
  }
  return true;
}

kalman_interfaces__msg__WheelStates *
kalman_interfaces__msg__WheelStates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__WheelStates * msg = (kalman_interfaces__msg__WheelStates *)allocator.allocate(sizeof(kalman_interfaces__msg__WheelStates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kalman_interfaces__msg__WheelStates));
  bool success = kalman_interfaces__msg__WheelStates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kalman_interfaces__msg__WheelStates__destroy(kalman_interfaces__msg__WheelStates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kalman_interfaces__msg__WheelStates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kalman_interfaces__msg__WheelStates__Sequence__init(kalman_interfaces__msg__WheelStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__WheelStates * data = NULL;

  if (size) {
    data = (kalman_interfaces__msg__WheelStates *)allocator.zero_allocate(size, sizeof(kalman_interfaces__msg__WheelStates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kalman_interfaces__msg__WheelStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kalman_interfaces__msg__WheelStates__fini(&data[i - 1]);
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
kalman_interfaces__msg__WheelStates__Sequence__fini(kalman_interfaces__msg__WheelStates__Sequence * array)
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
      kalman_interfaces__msg__WheelStates__fini(&array->data[i]);
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

kalman_interfaces__msg__WheelStates__Sequence *
kalman_interfaces__msg__WheelStates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__WheelStates__Sequence * array = (kalman_interfaces__msg__WheelStates__Sequence *)allocator.allocate(sizeof(kalman_interfaces__msg__WheelStates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kalman_interfaces__msg__WheelStates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kalman_interfaces__msg__WheelStates__Sequence__destroy(kalman_interfaces__msg__WheelStates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kalman_interfaces__msg__WheelStates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kalman_interfaces__msg__WheelStates__Sequence__are_equal(const kalman_interfaces__msg__WheelStates__Sequence * lhs, const kalman_interfaces__msg__WheelStates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kalman_interfaces__msg__WheelStates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kalman_interfaces__msg__WheelStates__Sequence__copy(
  const kalman_interfaces__msg__WheelStates__Sequence * input,
  kalman_interfaces__msg__WheelStates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kalman_interfaces__msg__WheelStates);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kalman_interfaces__msg__WheelStates * data =
      (kalman_interfaces__msg__WheelStates *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kalman_interfaces__msg__WheelStates__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kalman_interfaces__msg__WheelStates__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kalman_interfaces__msg__WheelStates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
