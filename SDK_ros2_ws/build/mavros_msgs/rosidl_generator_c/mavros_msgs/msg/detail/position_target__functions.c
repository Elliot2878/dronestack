// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/PositionTarget.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/position_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `acceleration_or_force`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mavros_msgs__msg__PositionTarget__init(mavros_msgs__msg__PositionTarget * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__PositionTarget__fini(msg);
    return false;
  }
  // coordinate_frame
  // type_mask
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    mavros_msgs__msg__PositionTarget__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    mavros_msgs__msg__PositionTarget__fini(msg);
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration_or_force)) {
    mavros_msgs__msg__PositionTarget__fini(msg);
    return false;
  }
  // yaw
  // yaw_rate
  return true;
}

void
mavros_msgs__msg__PositionTarget__fini(mavros_msgs__msg__PositionTarget * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // coordinate_frame
  // type_mask
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration_or_force
  geometry_msgs__msg__Vector3__fini(&msg->acceleration_or_force);
  // yaw
  // yaw_rate
}

bool
mavros_msgs__msg__PositionTarget__are_equal(const mavros_msgs__msg__PositionTarget * lhs, const mavros_msgs__msg__PositionTarget * rhs)
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
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
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
mavros_msgs__msg__PositionTarget__copy(
  const mavros_msgs__msg__PositionTarget * input,
  mavros_msgs__msg__PositionTarget * output)
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
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
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

mavros_msgs__msg__PositionTarget *
mavros_msgs__msg__PositionTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__PositionTarget * msg = (mavros_msgs__msg__PositionTarget *)allocator.allocate(sizeof(mavros_msgs__msg__PositionTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__PositionTarget));
  bool success = mavros_msgs__msg__PositionTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__PositionTarget__destroy(mavros_msgs__msg__PositionTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__PositionTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__PositionTarget__Sequence__init(mavros_msgs__msg__PositionTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__PositionTarget * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__PositionTarget *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__PositionTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__PositionTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__PositionTarget__fini(&data[i - 1]);
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
mavros_msgs__msg__PositionTarget__Sequence__fini(mavros_msgs__msg__PositionTarget__Sequence * array)
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
      mavros_msgs__msg__PositionTarget__fini(&array->data[i]);
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

mavros_msgs__msg__PositionTarget__Sequence *
mavros_msgs__msg__PositionTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__PositionTarget__Sequence * array = (mavros_msgs__msg__PositionTarget__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__PositionTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__PositionTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__PositionTarget__Sequence__destroy(mavros_msgs__msg__PositionTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__PositionTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__PositionTarget__Sequence__are_equal(const mavros_msgs__msg__PositionTarget__Sequence * lhs, const mavros_msgs__msg__PositionTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__PositionTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__PositionTarget__Sequence__copy(
  const mavros_msgs__msg__PositionTarget__Sequence * input,
  mavros_msgs__msg__PositionTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__PositionTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__PositionTarget * data =
      (mavros_msgs__msg__PositionTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__PositionTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__PositionTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__PositionTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
