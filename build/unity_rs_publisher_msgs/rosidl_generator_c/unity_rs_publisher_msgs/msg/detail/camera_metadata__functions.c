// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
// generated code does not contain a copyright notice
#include "unity_rs_publisher_msgs/msg/detail/camera_metadata__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unity_rs_publisher_msgs__msg__CameraMetadata__init(unity_rs_publisher_msgs__msg__CameraMetadata * msg)
{
  if (!msg) {
    return false;
  }
  // width
  // height
  // depth_min
  // depth_max
  // fx
  // fy
  // cx
  // cy
  return true;
}

void
unity_rs_publisher_msgs__msg__CameraMetadata__fini(unity_rs_publisher_msgs__msg__CameraMetadata * msg)
{
  if (!msg) {
    return;
  }
  // width
  // height
  // depth_min
  // depth_max
  // fx
  // fy
  // cx
  // cy
}

bool
unity_rs_publisher_msgs__msg__CameraMetadata__are_equal(const unity_rs_publisher_msgs__msg__CameraMetadata * lhs, const unity_rs_publisher_msgs__msg__CameraMetadata * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // depth_min
  if (lhs->depth_min != rhs->depth_min) {
    return false;
  }
  // depth_max
  if (lhs->depth_max != rhs->depth_max) {
    return false;
  }
  // fx
  if (lhs->fx != rhs->fx) {
    return false;
  }
  // fy
  if (lhs->fy != rhs->fy) {
    return false;
  }
  // cx
  if (lhs->cx != rhs->cx) {
    return false;
  }
  // cy
  if (lhs->cy != rhs->cy) {
    return false;
  }
  return true;
}

bool
unity_rs_publisher_msgs__msg__CameraMetadata__copy(
  const unity_rs_publisher_msgs__msg__CameraMetadata * input,
  unity_rs_publisher_msgs__msg__CameraMetadata * output)
{
  if (!input || !output) {
    return false;
  }
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // depth_min
  output->depth_min = input->depth_min;
  // depth_max
  output->depth_max = input->depth_max;
  // fx
  output->fx = input->fx;
  // fy
  output->fy = input->fy;
  // cx
  output->cx = input->cx;
  // cy
  output->cy = input->cy;
  return true;
}

unity_rs_publisher_msgs__msg__CameraMetadata *
unity_rs_publisher_msgs__msg__CameraMetadata__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unity_rs_publisher_msgs__msg__CameraMetadata * msg = (unity_rs_publisher_msgs__msg__CameraMetadata *)allocator.allocate(sizeof(unity_rs_publisher_msgs__msg__CameraMetadata), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unity_rs_publisher_msgs__msg__CameraMetadata));
  bool success = unity_rs_publisher_msgs__msg__CameraMetadata__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unity_rs_publisher_msgs__msg__CameraMetadata__destroy(unity_rs_publisher_msgs__msg__CameraMetadata * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unity_rs_publisher_msgs__msg__CameraMetadata__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__init(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unity_rs_publisher_msgs__msg__CameraMetadata * data = NULL;

  if (size) {
    data = (unity_rs_publisher_msgs__msg__CameraMetadata *)allocator.zero_allocate(size, sizeof(unity_rs_publisher_msgs__msg__CameraMetadata), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unity_rs_publisher_msgs__msg__CameraMetadata__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unity_rs_publisher_msgs__msg__CameraMetadata__fini(&data[i - 1]);
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
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__fini(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array)
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
      unity_rs_publisher_msgs__msg__CameraMetadata__fini(&array->data[i]);
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

unity_rs_publisher_msgs__msg__CameraMetadata__Sequence *
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array = (unity_rs_publisher_msgs__msg__CameraMetadata__Sequence *)allocator.allocate(sizeof(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__destroy(unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__are_equal(const unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * lhs, const unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unity_rs_publisher_msgs__msg__CameraMetadata__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unity_rs_publisher_msgs__msg__CameraMetadata__Sequence__copy(
  const unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * input,
  unity_rs_publisher_msgs__msg__CameraMetadata__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unity_rs_publisher_msgs__msg__CameraMetadata);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    unity_rs_publisher_msgs__msg__CameraMetadata * data =
      (unity_rs_publisher_msgs__msg__CameraMetadata *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unity_rs_publisher_msgs__msg__CameraMetadata__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          unity_rs_publisher_msgs__msg__CameraMetadata__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!unity_rs_publisher_msgs__msg__CameraMetadata__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
