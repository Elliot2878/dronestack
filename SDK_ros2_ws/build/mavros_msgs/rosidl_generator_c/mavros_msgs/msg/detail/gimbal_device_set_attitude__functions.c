// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/GimbalDeviceSetAttitude.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/gimbal_device_set_attitude__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `q`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
mavros_msgs__msg__GimbalDeviceSetAttitude__init(mavros_msgs__msg__GimbalDeviceSetAttitude * msg)
{
  if (!msg) {
    return false;
  }
  // target_system
  // target_component
  // flags
  // q
  if (!geometry_msgs__msg__Quaternion__init(&msg->q)) {
    mavros_msgs__msg__GimbalDeviceSetAttitude__fini(msg);
    return false;
  }
  // angular_velocity_x
  // angular_velocity_y
  // angular_velocity_z
  return true;
}

void
mavros_msgs__msg__GimbalDeviceSetAttitude__fini(mavros_msgs__msg__GimbalDeviceSetAttitude * msg)
{
  if (!msg) {
    return;
  }
  // target_system
  // target_component
  // flags
  // q
  geometry_msgs__msg__Quaternion__fini(&msg->q);
  // angular_velocity_x
  // angular_velocity_y
  // angular_velocity_z
}

bool
mavros_msgs__msg__GimbalDeviceSetAttitude__are_equal(const mavros_msgs__msg__GimbalDeviceSetAttitude * lhs, const mavros_msgs__msg__GimbalDeviceSetAttitude * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_system
  if (lhs->target_system != rhs->target_system) {
    return false;
  }
  // target_component
  if (lhs->target_component != rhs->target_component) {
    return false;
  }
  // flags
  if (lhs->flags != rhs->flags) {
    return false;
  }
  // q
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->q), &(rhs->q)))
  {
    return false;
  }
  // angular_velocity_x
  if (lhs->angular_velocity_x != rhs->angular_velocity_x) {
    return false;
  }
  // angular_velocity_y
  if (lhs->angular_velocity_y != rhs->angular_velocity_y) {
    return false;
  }
  // angular_velocity_z
  if (lhs->angular_velocity_z != rhs->angular_velocity_z) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__GimbalDeviceSetAttitude__copy(
  const mavros_msgs__msg__GimbalDeviceSetAttitude * input,
  mavros_msgs__msg__GimbalDeviceSetAttitude * output)
{
  if (!input || !output) {
    return false;
  }
  // target_system
  output->target_system = input->target_system;
  // target_component
  output->target_component = input->target_component;
  // flags
  output->flags = input->flags;
  // q
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->q), &(output->q)))
  {
    return false;
  }
  // angular_velocity_x
  output->angular_velocity_x = input->angular_velocity_x;
  // angular_velocity_y
  output->angular_velocity_y = input->angular_velocity_y;
  // angular_velocity_z
  output->angular_velocity_z = input->angular_velocity_z;
  return true;
}

mavros_msgs__msg__GimbalDeviceSetAttitude *
mavros_msgs__msg__GimbalDeviceSetAttitude__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GimbalDeviceSetAttitude * msg = (mavros_msgs__msg__GimbalDeviceSetAttitude *)allocator.allocate(sizeof(mavros_msgs__msg__GimbalDeviceSetAttitude), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__GimbalDeviceSetAttitude));
  bool success = mavros_msgs__msg__GimbalDeviceSetAttitude__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__GimbalDeviceSetAttitude__destroy(mavros_msgs__msg__GimbalDeviceSetAttitude * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__GimbalDeviceSetAttitude__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__init(mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GimbalDeviceSetAttitude * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__GimbalDeviceSetAttitude *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__GimbalDeviceSetAttitude), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__GimbalDeviceSetAttitude__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__GimbalDeviceSetAttitude__fini(&data[i - 1]);
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
mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__fini(mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * array)
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
      mavros_msgs__msg__GimbalDeviceSetAttitude__fini(&array->data[i]);
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

mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence *
mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * array = (mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__destroy(mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__are_equal(const mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * lhs, const mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__GimbalDeviceSetAttitude__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence__copy(
  const mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * input,
  mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__GimbalDeviceSetAttitude);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__GimbalDeviceSetAttitude * data =
      (mavros_msgs__msg__GimbalDeviceSetAttitude *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__GimbalDeviceSetAttitude__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__GimbalDeviceSetAttitude__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__GimbalDeviceSetAttitude__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
