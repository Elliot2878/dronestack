// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/WaypointList.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/waypoint_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `waypoints`
#include "mavros_msgs/msg/detail/waypoint__functions.h"

bool
mavros_msgs__msg__WaypointList__init(mavros_msgs__msg__WaypointList * msg)
{
  if (!msg) {
    return false;
  }
  // current_seq
  // waypoints
  if (!mavros_msgs__msg__Waypoint__Sequence__init(&msg->waypoints, 0)) {
    mavros_msgs__msg__WaypointList__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__WaypointList__fini(mavros_msgs__msg__WaypointList * msg)
{
  if (!msg) {
    return;
  }
  // current_seq
  // waypoints
  mavros_msgs__msg__Waypoint__Sequence__fini(&msg->waypoints);
}

bool
mavros_msgs__msg__WaypointList__are_equal(const mavros_msgs__msg__WaypointList * lhs, const mavros_msgs__msg__WaypointList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_seq
  if (lhs->current_seq != rhs->current_seq) {
    return false;
  }
  // waypoints
  if (!mavros_msgs__msg__Waypoint__Sequence__are_equal(
      &(lhs->waypoints), &(rhs->waypoints)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__WaypointList__copy(
  const mavros_msgs__msg__WaypointList * input,
  mavros_msgs__msg__WaypointList * output)
{
  if (!input || !output) {
    return false;
  }
  // current_seq
  output->current_seq = input->current_seq;
  // waypoints
  if (!mavros_msgs__msg__Waypoint__Sequence__copy(
      &(input->waypoints), &(output->waypoints)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__WaypointList *
mavros_msgs__msg__WaypointList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__WaypointList * msg = (mavros_msgs__msg__WaypointList *)allocator.allocate(sizeof(mavros_msgs__msg__WaypointList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__WaypointList));
  bool success = mavros_msgs__msg__WaypointList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__WaypointList__destroy(mavros_msgs__msg__WaypointList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__WaypointList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__WaypointList__Sequence__init(mavros_msgs__msg__WaypointList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__WaypointList * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__WaypointList *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__WaypointList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__WaypointList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__WaypointList__fini(&data[i - 1]);
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
mavros_msgs__msg__WaypointList__Sequence__fini(mavros_msgs__msg__WaypointList__Sequence * array)
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
      mavros_msgs__msg__WaypointList__fini(&array->data[i]);
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

mavros_msgs__msg__WaypointList__Sequence *
mavros_msgs__msg__WaypointList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__WaypointList__Sequence * array = (mavros_msgs__msg__WaypointList__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__WaypointList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__WaypointList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__WaypointList__Sequence__destroy(mavros_msgs__msg__WaypointList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__WaypointList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__WaypointList__Sequence__are_equal(const mavros_msgs__msg__WaypointList__Sequence * lhs, const mavros_msgs__msg__WaypointList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__WaypointList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__WaypointList__Sequence__copy(
  const mavros_msgs__msg__WaypointList__Sequence * input,
  mavros_msgs__msg__WaypointList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__WaypointList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__WaypointList * data =
      (mavros_msgs__msg__WaypointList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__WaypointList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__WaypointList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__WaypointList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
