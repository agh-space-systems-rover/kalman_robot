// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from kalman_interfaces:msg/MasterMessage.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/msg/detail/master_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
kalman_interfaces__msg__MasterMessage__init(kalman_interfaces__msg__MasterMessage * msg)
{
  if (!msg) {
    return false;
  }
  // cmd
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    kalman_interfaces__msg__MasterMessage__fini(msg);
    return false;
  }
  return true;
}

void
kalman_interfaces__msg__MasterMessage__fini(kalman_interfaces__msg__MasterMessage * msg)
{
  if (!msg) {
    return;
  }
  // cmd
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
kalman_interfaces__msg__MasterMessage__are_equal(const kalman_interfaces__msg__MasterMessage * lhs, const kalman_interfaces__msg__MasterMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd
  if (lhs->cmd != rhs->cmd) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
kalman_interfaces__msg__MasterMessage__copy(
  const kalman_interfaces__msg__MasterMessage * input,
  kalman_interfaces__msg__MasterMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd
  output->cmd = input->cmd;
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

kalman_interfaces__msg__MasterMessage *
kalman_interfaces__msg__MasterMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__MasterMessage * msg = (kalman_interfaces__msg__MasterMessage *)allocator.allocate(sizeof(kalman_interfaces__msg__MasterMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kalman_interfaces__msg__MasterMessage));
  bool success = kalman_interfaces__msg__MasterMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kalman_interfaces__msg__MasterMessage__destroy(kalman_interfaces__msg__MasterMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kalman_interfaces__msg__MasterMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kalman_interfaces__msg__MasterMessage__Sequence__init(kalman_interfaces__msg__MasterMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__MasterMessage * data = NULL;

  if (size) {
    data = (kalman_interfaces__msg__MasterMessage *)allocator.zero_allocate(size, sizeof(kalman_interfaces__msg__MasterMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kalman_interfaces__msg__MasterMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kalman_interfaces__msg__MasterMessage__fini(&data[i - 1]);
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
kalman_interfaces__msg__MasterMessage__Sequence__fini(kalman_interfaces__msg__MasterMessage__Sequence * array)
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
      kalman_interfaces__msg__MasterMessage__fini(&array->data[i]);
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

kalman_interfaces__msg__MasterMessage__Sequence *
kalman_interfaces__msg__MasterMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__msg__MasterMessage__Sequence * array = (kalman_interfaces__msg__MasterMessage__Sequence *)allocator.allocate(sizeof(kalman_interfaces__msg__MasterMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kalman_interfaces__msg__MasterMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kalman_interfaces__msg__MasterMessage__Sequence__destroy(kalman_interfaces__msg__MasterMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kalman_interfaces__msg__MasterMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kalman_interfaces__msg__MasterMessage__Sequence__are_equal(const kalman_interfaces__msg__MasterMessage__Sequence * lhs, const kalman_interfaces__msg__MasterMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kalman_interfaces__msg__MasterMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kalman_interfaces__msg__MasterMessage__Sequence__copy(
  const kalman_interfaces__msg__MasterMessage__Sequence * input,
  kalman_interfaces__msg__MasterMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kalman_interfaces__msg__MasterMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kalman_interfaces__msg__MasterMessage * data =
      (kalman_interfaces__msg__MasterMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kalman_interfaces__msg__MasterMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kalman_interfaces__msg__MasterMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kalman_interfaces__msg__MasterMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
