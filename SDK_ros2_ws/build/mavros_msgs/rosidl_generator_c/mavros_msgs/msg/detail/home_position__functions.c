// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/HomePosition.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/home_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `geo`
#include "geographic_msgs/msg/detail/geo_point__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `approach`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mavros_msgs__msg__HomePosition__init(mavros_msgs__msg__HomePosition * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__HomePosition__fini(msg);
    return false;
  }
  // geo
  if (!geographic_msgs__msg__GeoPoint__init(&msg->geo)) {
    mavros_msgs__msg__HomePosition__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    mavros_msgs__msg__HomePosition__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    mavros_msgs__msg__HomePosition__fini(msg);
    return false;
  }
  // approach
  if (!geometry_msgs__msg__Vector3__init(&msg->approach)) {
    mavros_msgs__msg__HomePosition__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__HomePosition__fini(mavros_msgs__msg__HomePosition * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // geo
  geographic_msgs__msg__GeoPoint__fini(&msg->geo);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // approach
  geometry_msgs__msg__Vector3__fini(&msg->approach);
}

bool
mavros_msgs__msg__HomePosition__are_equal(const mavros_msgs__msg__HomePosition * lhs, const mavros_msgs__msg__HomePosition * rhs)
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
  // geo
  if (!geographic_msgs__msg__GeoPoint__are_equal(
      &(lhs->geo), &(rhs->geo)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // approach
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->approach), &(rhs->approach)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__HomePosition__copy(
  const mavros_msgs__msg__HomePosition * input,
  mavros_msgs__msg__HomePosition * output)
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
  // geo
  if (!geographic_msgs__msg__GeoPoint__copy(
      &(input->geo), &(output->geo)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // approach
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->approach), &(output->approach)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__HomePosition *
mavros_msgs__msg__HomePosition__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HomePosition * msg = (mavros_msgs__msg__HomePosition *)allocator.allocate(sizeof(mavros_msgs__msg__HomePosition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__HomePosition));
  bool success = mavros_msgs__msg__HomePosition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__HomePosition__destroy(mavros_msgs__msg__HomePosition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__HomePosition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__HomePosition__Sequence__init(mavros_msgs__msg__HomePosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HomePosition * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__HomePosition *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__HomePosition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__HomePosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__HomePosition__fini(&data[i - 1]);
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
mavros_msgs__msg__HomePosition__Sequence__fini(mavros_msgs__msg__HomePosition__Sequence * array)
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
      mavros_msgs__msg__HomePosition__fini(&array->data[i]);
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

mavros_msgs__msg__HomePosition__Sequence *
mavros_msgs__msg__HomePosition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HomePosition__Sequence * array = (mavros_msgs__msg__HomePosition__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__HomePosition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__HomePosition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__HomePosition__Sequence__destroy(mavros_msgs__msg__HomePosition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__HomePosition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__HomePosition__Sequence__are_equal(const mavros_msgs__msg__HomePosition__Sequence * lhs, const mavros_msgs__msg__HomePosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__HomePosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__HomePosition__Sequence__copy(
  const mavros_msgs__msg__HomePosition__Sequence * input,
  mavros_msgs__msg__HomePosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__HomePosition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__HomePosition * data =
      (mavros_msgs__msg__HomePosition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__HomePosition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__HomePosition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__HomePosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
