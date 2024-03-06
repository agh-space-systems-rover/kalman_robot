// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice
#include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
kalman_interfaces__srv__CalibrateCompass_Request__init(kalman_interfaces__srv__CalibrateCompass_Request * msg)
{
  if (!msg) {
    return false;
  }
  // duration
  msg->duration = 30.0f;
  // angular_velocity
  msg->angular_velocity = 0.5f;
  return true;
}

void
kalman_interfaces__srv__CalibrateCompass_Request__fini(kalman_interfaces__srv__CalibrateCompass_Request * msg)
{
  if (!msg) {
    return;
  }
  // duration
  // angular_velocity
}

bool
kalman_interfaces__srv__CalibrateCompass_Request__are_equal(const kalman_interfaces__srv__CalibrateCompass_Request * lhs, const kalman_interfaces__srv__CalibrateCompass_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  return true;
}

bool
kalman_interfaces__srv__CalibrateCompass_Request__copy(
  const kalman_interfaces__srv__CalibrateCompass_Request * input,
  kalman_interfaces__srv__CalibrateCompass_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // duration
  output->duration = input->duration;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  return true;
}

kalman_interfaces__srv__CalibrateCompass_Request *
kalman_interfaces__srv__CalibrateCompass_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__srv__CalibrateCompass_Request * msg = (kalman_interfaces__srv__CalibrateCompass_Request *)allocator.allocate(sizeof(kalman_interfaces__srv__CalibrateCompass_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kalman_interfaces__srv__CalibrateCompass_Request));
  bool success = kalman_interfaces__srv__CalibrateCompass_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kalman_interfaces__srv__CalibrateCompass_Request__destroy(kalman_interfaces__srv__CalibrateCompass_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kalman_interfaces__srv__CalibrateCompass_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kalman_interfaces__srv__CalibrateCompass_Request__Sequence__init(kalman_interfaces__srv__CalibrateCompass_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__srv__CalibrateCompass_Request * data = NULL;

  if (size) {
    data = (kalman_interfaces__srv__CalibrateCompass_Request *)allocator.zero_allocate(size, sizeof(kalman_interfaces__srv__CalibrateCompass_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kalman_interfaces__srv__CalibrateCompass_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kalman_interfaces__srv__CalibrateCompass_Request__fini(&data[i - 1]);
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
kalman_interfaces__srv__CalibrateCompass_Request__Sequence__fini(kalman_interfaces__srv__CalibrateCompass_Request__Sequence * array)
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
      kalman_interfaces__srv__CalibrateCompass_Request__fini(&array->data[i]);
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

kalman_interfaces__srv__CalibrateCompass_Request__Sequence *
kalman_interfaces__srv__CalibrateCompass_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__srv__CalibrateCompass_Request__Sequence * array = (kalman_interfaces__srv__CalibrateCompass_Request__Sequence *)allocator.allocate(sizeof(kalman_interfaces__srv__CalibrateCompass_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kalman_interfaces__srv__CalibrateCompass_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kalman_interfaces__srv__CalibrateCompass_Request__Sequence__destroy(kalman_interfaces__srv__CalibrateCompass_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kalman_interfaces__srv__CalibrateCompass_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kalman_interfaces__srv__CalibrateCompass_Request__Sequence__are_equal(const kalman_interfaces__srv__CalibrateCompass_Request__Sequence * lhs, const kalman_interfaces__srv__CalibrateCompass_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kalman_interfaces__srv__CalibrateCompass_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kalman_interfaces__srv__CalibrateCompass_Request__Sequence__copy(
  const kalman_interfaces__srv__CalibrateCompass_Request__Sequence * input,
  kalman_interfaces__srv__CalibrateCompass_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kalman_interfaces__srv__CalibrateCompass_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kalman_interfaces__srv__CalibrateCompass_Request * data =
      (kalman_interfaces__srv__CalibrateCompass_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kalman_interfaces__srv__CalibrateCompass_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kalman_interfaces__srv__CalibrateCompass_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kalman_interfaces__srv__CalibrateCompass_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
kalman_interfaces__srv__CalibrateCompass_Response__init(kalman_interfaces__srv__CalibrateCompass_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // cc_mag_field
  // cc_offset0
  // cc_offset1
  // cc_offset2
  // cc_gain0
  // cc_gain1
  // cc_gain2
  // cc_t0
  // cc_t1
  // cc_t2
  // cc_t3
  // cc_t4
  // cc_t5
  return true;
}

void
kalman_interfaces__srv__CalibrateCompass_Response__fini(kalman_interfaces__srv__CalibrateCompass_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // cc_mag_field
  // cc_offset0
  // cc_offset1
  // cc_offset2
  // cc_gain0
  // cc_gain1
  // cc_gain2
  // cc_t0
  // cc_t1
  // cc_t2
  // cc_t3
  // cc_t4
  // cc_t5
}

bool
kalman_interfaces__srv__CalibrateCompass_Response__are_equal(const kalman_interfaces__srv__CalibrateCompass_Response * lhs, const kalman_interfaces__srv__CalibrateCompass_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // cc_mag_field
  if (lhs->cc_mag_field != rhs->cc_mag_field) {
    return false;
  }
  // cc_offset0
  if (lhs->cc_offset0 != rhs->cc_offset0) {
    return false;
  }
  // cc_offset1
  if (lhs->cc_offset1 != rhs->cc_offset1) {
    return false;
  }
  // cc_offset2
  if (lhs->cc_offset2 != rhs->cc_offset2) {
    return false;
  }
  // cc_gain0
  if (lhs->cc_gain0 != rhs->cc_gain0) {
    return false;
  }
  // cc_gain1
  if (lhs->cc_gain1 != rhs->cc_gain1) {
    return false;
  }
  // cc_gain2
  if (lhs->cc_gain2 != rhs->cc_gain2) {
    return false;
  }
  // cc_t0
  if (lhs->cc_t0 != rhs->cc_t0) {
    return false;
  }
  // cc_t1
  if (lhs->cc_t1 != rhs->cc_t1) {
    return false;
  }
  // cc_t2
  if (lhs->cc_t2 != rhs->cc_t2) {
    return false;
  }
  // cc_t3
  if (lhs->cc_t3 != rhs->cc_t3) {
    return false;
  }
  // cc_t4
  if (lhs->cc_t4 != rhs->cc_t4) {
    return false;
  }
  // cc_t5
  if (lhs->cc_t5 != rhs->cc_t5) {
    return false;
  }
  return true;
}

bool
kalman_interfaces__srv__CalibrateCompass_Response__copy(
  const kalman_interfaces__srv__CalibrateCompass_Response * input,
  kalman_interfaces__srv__CalibrateCompass_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // cc_mag_field
  output->cc_mag_field = input->cc_mag_field;
  // cc_offset0
  output->cc_offset0 = input->cc_offset0;
  // cc_offset1
  output->cc_offset1 = input->cc_offset1;
  // cc_offset2
  output->cc_offset2 = input->cc_offset2;
  // cc_gain0
  output->cc_gain0 = input->cc_gain0;
  // cc_gain1
  output->cc_gain1 = input->cc_gain1;
  // cc_gain2
  output->cc_gain2 = input->cc_gain2;
  // cc_t0
  output->cc_t0 = input->cc_t0;
  // cc_t1
  output->cc_t1 = input->cc_t1;
  // cc_t2
  output->cc_t2 = input->cc_t2;
  // cc_t3
  output->cc_t3 = input->cc_t3;
  // cc_t4
  output->cc_t4 = input->cc_t4;
  // cc_t5
  output->cc_t5 = input->cc_t5;
  return true;
}

kalman_interfaces__srv__CalibrateCompass_Response *
kalman_interfaces__srv__CalibrateCompass_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__srv__CalibrateCompass_Response * msg = (kalman_interfaces__srv__CalibrateCompass_Response *)allocator.allocate(sizeof(kalman_interfaces__srv__CalibrateCompass_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kalman_interfaces__srv__CalibrateCompass_Response));
  bool success = kalman_interfaces__srv__CalibrateCompass_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kalman_interfaces__srv__CalibrateCompass_Response__destroy(kalman_interfaces__srv__CalibrateCompass_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kalman_interfaces__srv__CalibrateCompass_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kalman_interfaces__srv__CalibrateCompass_Response__Sequence__init(kalman_interfaces__srv__CalibrateCompass_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__srv__CalibrateCompass_Response * data = NULL;

  if (size) {
    data = (kalman_interfaces__srv__CalibrateCompass_Response *)allocator.zero_allocate(size, sizeof(kalman_interfaces__srv__CalibrateCompass_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kalman_interfaces__srv__CalibrateCompass_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kalman_interfaces__srv__CalibrateCompass_Response__fini(&data[i - 1]);
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
kalman_interfaces__srv__CalibrateCompass_Response__Sequence__fini(kalman_interfaces__srv__CalibrateCompass_Response__Sequence * array)
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
      kalman_interfaces__srv__CalibrateCompass_Response__fini(&array->data[i]);
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

kalman_interfaces__srv__CalibrateCompass_Response__Sequence *
kalman_interfaces__srv__CalibrateCompass_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kalman_interfaces__srv__CalibrateCompass_Response__Sequence * array = (kalman_interfaces__srv__CalibrateCompass_Response__Sequence *)allocator.allocate(sizeof(kalman_interfaces__srv__CalibrateCompass_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kalman_interfaces__srv__CalibrateCompass_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kalman_interfaces__srv__CalibrateCompass_Response__Sequence__destroy(kalman_interfaces__srv__CalibrateCompass_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kalman_interfaces__srv__CalibrateCompass_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kalman_interfaces__srv__CalibrateCompass_Response__Sequence__are_equal(const kalman_interfaces__srv__CalibrateCompass_Response__Sequence * lhs, const kalman_interfaces__srv__CalibrateCompass_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kalman_interfaces__srv__CalibrateCompass_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kalman_interfaces__srv__CalibrateCompass_Response__Sequence__copy(
  const kalman_interfaces__srv__CalibrateCompass_Response__Sequence * input,
  kalman_interfaces__srv__CalibrateCompass_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kalman_interfaces__srv__CalibrateCompass_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kalman_interfaces__srv__CalibrateCompass_Response * data =
      (kalman_interfaces__srv__CalibrateCompass_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kalman_interfaces__srv__CalibrateCompass_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kalman_interfaces__srv__CalibrateCompass_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kalman_interfaces__srv__CalibrateCompass_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
