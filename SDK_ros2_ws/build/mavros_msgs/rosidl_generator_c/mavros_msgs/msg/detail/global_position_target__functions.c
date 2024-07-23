// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/GlobalPositionTarget.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/global_position_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `velocity`
// Member `acceleration_or_force`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mavros_msgs__msg__GlobalPositionTarget__init(mavros_msgs__msg__GlobalPositionTarget * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__GlobalPositionTarget__fini(msg);
    return false;
  }
  // coordinate_frame
  // type_mask
  // latitude
  // longitude
  // altitude
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    mavros_msgs__msg__GlobalPositionTarget__fini(msg);
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration_or_force)) {
    mavros_msgs__msg__GlobalPositionTarget__fini(msg);
    return false;
  }
  // yaw
  // yaw_rate
  return true;
}

void
mavros_msgs__msg__GlobalPositionTarget__fini(mavros_msgs__msg__GlobalPositionTarget * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // coordinate_frame
  // type_mask
  // latitude
  // longitude
  // altitude
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration_or_force
  geometry_msgs__msg__Vector3__fini(&msg->acceleration_or_force);
  // yaw
  // yaw_rate
}

bool
mavros_msgs__msg__GlobalPositionTarget__are_equal(const mavros_msgs__msg__GlobalPositionTarget * lhs, const mavros_msgs__msg__GlobalPositionTarget * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // coordinate_frame
  if (lhs->coordinate_frame != rhs->coordinate_frame) {
    return false;
  }
  // type_mask
  if (lhs->type_mask != rhs->type_mask) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration_or_force), &(rhs->acceleration_or_force)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__GlobalPositionTarget__copy(
  const mavros_msgs__msg__GlobalPositionTarget * input,
  mavros_msgs__msg__GlobalPositionTarget * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // coordinate_frame
  output->coordinate_frame = input->coordinate_frame;
  // type_mask
  output->type_mask = input->type_mask;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration_or_force), &(output->acceleration_or_force)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  return true;
}

mavros_msgs__msg__GlobalPositionTarget *
mavros_msgs__msg__GlobalPositionTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GlobalPositionTarget * msg = (mavros_msgs__msg__GlobalPositionTarget *)allocator.allocate(sizeof(mavros_msgs__msg__GlobalPositionTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__GlobalPositionTarget));
  bool success = mavros_msgs__msg__GlobalPositionTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__GlobalPositionTarget__destroy(mavros_msgs__msg__GlobalPositionTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__GlobalPositionTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__GlobalPositionTarget__Sequence__init(mavros_msgs__msg__GlobalPositionTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GlobalPositionTarget * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__GlobalPositionTarget *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__GlobalPositionTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__GlobalPositionTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__GlobalPositionTarget__fini(&data[i - 1]);
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
mavros_msgs__msg__GlobalPositionTarget__Sequence__fini(mavros_msgs__msg__GlobalPositionTarget__Sequence * array)
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
      mavros_msgs__msg__GlobalPositionTarget__fini(&array->data[i]);
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

mavros_msgs__msg__GlobalPositionTarget__Sequence *
mavros_msgs__msg__GlobalPositionTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GlobalPositionTarget__Sequence * array = (mavros_msgs__msg__GlobalPositionTarget__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__GlobalPositionTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__GlobalPositionTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__GlobalPositionTarget__Sequence__destroy(mavros_msgs__msg__GlobalPositionTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__GlobalPositionTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__GlobalPositionTarget__Sequence__are_equal(const mavros_msgs__msg__GlobalPositionTarget__Sequence * lhs, const mavros_msgs__msg__GlobalPositionTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__GlobalPositionTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__GlobalPositionTarget__Sequence__copy(
  const mavros_msgs__msg__GlobalPositionTarget__Sequence * input,
  mavros_msgs__msg__GlobalPositionTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__GlobalPositionTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__GlobalPositionTarget * data =
      (mavros_msgs__msg__GlobalPositionTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__GlobalPositionTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__GlobalPositionTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__GlobalPositionTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
