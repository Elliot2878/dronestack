// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/LandingTarget.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/landing_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
mavros_msgs__msg__LandingTarget__init(mavros_msgs__msg__LandingTarget * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__LandingTarget__fini(msg);
    return false;
  }
  // target_num
  // frame
  // angle
  // distance
  // size
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    mavros_msgs__msg__LandingTarget__fini(msg);
    return false;
  }
  // type
  return true;
}

void
mavros_msgs__msg__LandingTarget__fini(mavros_msgs__msg__LandingTarget * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // target_num
  // frame
  // angle
  // distance
  // size
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // type
}

bool
mavros_msgs__msg__LandingTarget__are_equal(const mavros_msgs__msg__LandingTarget * lhs, const mavros_msgs__msg__LandingTarget * rhs)
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
  // target_num
  if (lhs->target_num != rhs->target_num) {
    return false;
  }
  // frame
  if (lhs->frame != rhs->frame) {
    return false;
  }
  // angle
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->angle[i] != rhs->angle[i]) {
      return false;
    }
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // size
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->size[i] != rhs->size[i]) {
      return false;
    }
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__LandingTarget__copy(
  const mavros_msgs__msg__LandingTarget * input,
  mavros_msgs__msg__LandingTarget * output)
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
  // target_num
  output->target_num = input->target_num;
  // frame
  output->frame = input->frame;
  // angle
  for (size_t i = 0; i < 2; ++i) {
    output->angle[i] = input->angle[i];
  }
  // distance
  output->distance = input->distance;
  // size
  for (size_t i = 0; i < 2; ++i) {
    output->size[i] = input->size[i];
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // type
  output->type = input->type;
  return true;
}

mavros_msgs__msg__LandingTarget *
mavros_msgs__msg__LandingTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__LandingTarget * msg = (mavros_msgs__msg__LandingTarget *)allocator.allocate(sizeof(mavros_msgs__msg__LandingTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__LandingTarget));
  bool success = mavros_msgs__msg__LandingTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__LandingTarget__destroy(mavros_msgs__msg__LandingTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__LandingTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__LandingTarget__Sequence__init(mavros_msgs__msg__LandingTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__LandingTarget * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__LandingTarget *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__LandingTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__LandingTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__LandingTarget__fini(&data[i - 1]);
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
mavros_msgs__msg__LandingTarget__Sequence__fini(mavros_msgs__msg__LandingTarget__Sequence * array)
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
      mavros_msgs__msg__LandingTarget__fini(&array->data[i]);
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

mavros_msgs__msg__LandingTarget__Sequence *
mavros_msgs__msg__LandingTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__LandingTarget__Sequence * array = (mavros_msgs__msg__LandingTarget__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__LandingTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__LandingTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__LandingTarget__Sequence__destroy(mavros_msgs__msg__LandingTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__LandingTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__LandingTarget__Sequence__are_equal(const mavros_msgs__msg__LandingTarget__Sequence * lhs, const mavros_msgs__msg__LandingTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__LandingTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__LandingTarget__Sequence__copy(
  const mavros_msgs__msg__LandingTarget__Sequence * input,
  mavros_msgs__msg__LandingTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__LandingTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__LandingTarget * data =
      (mavros_msgs__msg__LandingTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__LandingTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__LandingTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__LandingTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
