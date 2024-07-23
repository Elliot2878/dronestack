// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `point_1`
// Member `point_2`
// Member `point_3`
// Member `point_4`
// Member `point_5`
#include "mavros_msgs/msg/detail/position_target__functions.h"

bool
mavros_msgs__msg__Trajectory__init(mavros_msgs__msg__Trajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // type
  // point_1
  if (!mavros_msgs__msg__PositionTarget__init(&msg->point_1)) {
    mavros_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // point_2
  if (!mavros_msgs__msg__PositionTarget__init(&msg->point_2)) {
    mavros_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // point_3
  if (!mavros_msgs__msg__PositionTarget__init(&msg->point_3)) {
    mavros_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // point_4
  if (!mavros_msgs__msg__PositionTarget__init(&msg->point_4)) {
    mavros_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // point_5
  if (!mavros_msgs__msg__PositionTarget__init(&msg->point_5)) {
    mavros_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // point_valid
  // command
  // time_horizon
  return true;
}

void
mavros_msgs__msg__Trajectory__fini(mavros_msgs__msg__Trajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // type
  // point_1
  mavros_msgs__msg__PositionTarget__fini(&msg->point_1);
  // point_2
  mavros_msgs__msg__PositionTarget__fini(&msg->point_2);
  // point_3
  mavros_msgs__msg__PositionTarget__fini(&msg->point_3);
  // point_4
  mavros_msgs__msg__PositionTarget__fini(&msg->point_4);
  // point_5
  mavros_msgs__msg__PositionTarget__fini(&msg->point_5);
  // point_valid
  // command
  // time_horizon
}

bool
mavros_msgs__msg__Trajectory__are_equal(const mavros_msgs__msg__Trajectory * lhs, const mavros_msgs__msg__Trajectory * rhs)
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
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // point_1
  if (!mavros_msgs__msg__PositionTarget__are_equal(
      &(lhs->point_1), &(rhs->point_1)))
  {
    return false;
  }
  // point_2
  if (!mavros_msgs__msg__PositionTarget__are_equal(
      &(lhs->point_2), &(rhs->point_2)))
  {
    return false;
  }
  // point_3
  if (!mavros_msgs__msg__PositionTarget__are_equal(
      &(lhs->point_3), &(rhs->point_3)))
  {
    return false;
  }
  // point_4
  if (!mavros_msgs__msg__PositionTarget__are_equal(
      &(lhs->point_4), &(rhs->point_4)))
  {
    return false;
  }
  // point_5
  if (!mavros_msgs__msg__PositionTarget__are_equal(
      &(lhs->point_5), &(rhs->point_5)))
  {
    return false;
  }
  // point_valid
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->point_valid[i] != rhs->point_valid[i]) {
      return false;
    }
  }
  // command
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->command[i] != rhs->command[i]) {
      return false;
    }
  }
  // time_horizon
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->time_horizon[i] != rhs->time_horizon[i]) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Trajectory__copy(
  const mavros_msgs__msg__Trajectory * input,
  mavros_msgs__msg__Trajectory * output)
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
  // type
  output->type = input->type;
  // point_1
  if (!mavros_msgs__msg__PositionTarget__copy(
      &(input->point_1), &(output->point_1)))
  {
    return false;
  }
  // point_2
  if (!mavros_msgs__msg__PositionTarget__copy(
      &(input->point_2), &(output->point_2)))
  {
    return false;
  }
  // point_3
  if (!mavros_msgs__msg__PositionTarget__copy(
      &(input->point_3), &(output->point_3)))
  {
    return false;
  }
  // point_4
  if (!mavros_msgs__msg__PositionTarget__copy(
      &(input->point_4), &(output->point_4)))
  {
    return false;
  }
  // point_5
  if (!mavros_msgs__msg__PositionTarget__copy(
      &(input->point_5), &(output->point_5)))
  {
    return false;
  }
  // point_valid
  for (size_t i = 0; i < 5; ++i) {
    output->point_valid[i] = input->point_valid[i];
  }
  // command
  for (size_t i = 0; i < 5; ++i) {
    output->command[i] = input->command[i];
  }
  // time_horizon
  for (size_t i = 0; i < 5; ++i) {
    output->time_horizon[i] = input->time_horizon[i];
  }
  return true;
}

mavros_msgs__msg__Trajectory *
mavros_msgs__msg__Trajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Trajectory * msg = (mavros_msgs__msg__Trajectory *)allocator.allocate(sizeof(mavros_msgs__msg__Trajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__Trajectory));
  bool success = mavros_msgs__msg__Trajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__Trajectory__destroy(mavros_msgs__msg__Trajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__Trajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__Trajectory__Sequence__init(mavros_msgs__msg__Trajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Trajectory * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__Trajectory *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__Trajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__Trajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__Trajectory__fini(&data[i - 1]);
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
mavros_msgs__msg__Trajectory__Sequence__fini(mavros_msgs__msg__Trajectory__Sequence * array)
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
      mavros_msgs__msg__Trajectory__fini(&array->data[i]);
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

mavros_msgs__msg__Trajectory__Sequence *
mavros_msgs__msg__Trajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Trajectory__Sequence * array = (mavros_msgs__msg__Trajectory__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__Trajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__Trajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__Trajectory__Sequence__destroy(mavros_msgs__msg__Trajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__Trajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__Trajectory__Sequence__are_equal(const mavros_msgs__msg__Trajectory__Sequence * lhs, const mavros_msgs__msg__Trajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__Trajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Trajectory__Sequence__copy(
  const mavros_msgs__msg__Trajectory__Sequence * input,
  mavros_msgs__msg__Trajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__Trajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__Trajectory * data =
      (mavros_msgs__msg__Trajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__Trajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__Trajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__Trajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
