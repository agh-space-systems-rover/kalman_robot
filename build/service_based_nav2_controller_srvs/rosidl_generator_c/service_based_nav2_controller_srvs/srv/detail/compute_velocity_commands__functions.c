// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `path`
#include "nav_msgs/msg/detail/path__functions.h"

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__init(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(msg);
    return false;
  }
  // path
  if (!nav_msgs__msg__Path__init(&msg->path)) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(msg);
    return false;
  }
  return true;
}

void
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
  // path
  nav_msgs__msg__Path__fini(&msg->path);
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__are_equal(const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * lhs, const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // path
  if (!nav_msgs__msg__Path__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  return true;
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__copy(
  const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * input,
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // path
  if (!nav_msgs__msg__Path__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  return true;
}

service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request *
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * msg = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request *)allocator.allocate(sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request));
  bool success = service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__destroy(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__init(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * data = NULL;

  if (size) {
    data = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request *)allocator.zero_allocate(size, sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(&data[i - 1]);
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
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__fini(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * array)
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
      service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(&array->data[i]);
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

service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence *
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * array = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence *)allocator.allocate(sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__destroy(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__are_equal(const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * lhs, const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence__copy(
  const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * input,
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * data =
      (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `cmd_vel`
#include "geometry_msgs/msg/detail/twist_stamped__functions.h"

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__init(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_vel
  if (!geometry_msgs__msg__TwistStamped__init(&msg->cmd_vel)) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(msg);
    return false;
  }
  return true;
}

void
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * msg)
{
  if (!msg) {
    return;
  }
  // cmd_vel
  geometry_msgs__msg__TwistStamped__fini(&msg->cmd_vel);
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__are_equal(const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * lhs, const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_vel
  if (!geometry_msgs__msg__TwistStamped__are_equal(
      &(lhs->cmd_vel), &(rhs->cmd_vel)))
  {
    return false;
  }
  return true;
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__copy(
  const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * input,
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_vel
  if (!geometry_msgs__msg__TwistStamped__copy(
      &(input->cmd_vel), &(output->cmd_vel)))
  {
    return false;
  }
  return true;
}

service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response *
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * msg = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response *)allocator.allocate(sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response));
  bool success = service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__destroy(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__init(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * data = NULL;

  if (size) {
    data = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response *)allocator.zero_allocate(size, sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(&data[i - 1]);
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
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__fini(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * array)
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
      service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(&array->data[i]);
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

service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence *
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * array = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence *)allocator.allocate(sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__destroy(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__are_equal(const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * lhs, const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence__copy(
  const service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * input,
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * data =
      (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
