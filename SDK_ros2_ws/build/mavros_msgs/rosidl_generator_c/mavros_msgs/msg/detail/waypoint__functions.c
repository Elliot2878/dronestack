// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/waypoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mavros_msgs__msg__Waypoint__init(mavros_msgs__msg__Waypoint * msg)
{
  if (!msg) {
    return false;
  }
  // frame
  // command
  // is_current
  // autocontinue
  // param1
  // param2
  // param3
  // param4
  // x_lat
  // y_long
  // z_alt
  return true;
}

void
mavros_msgs__msg__Waypoint__fini(mavros_msgs__msg__Waypoint * msg)
{
  if (!msg) {
    return;
  }
  // frame
  // command
  // is_current
  // autocontinue
  // param1
  // param2
  // param3
  // param4
  // x_lat
  // y_long
  // z_alt
}

bool
mavros_msgs__msg__Waypoint__are_equal(const mavros_msgs__msg__Waypoint * lhs, const mavros_msgs__msg__Waypoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame
  if (lhs->frame != rhs->frame) {
    return false;
  }
  // command
  if (lhs->command != rhs->command) {
    return false;
  }
  // is_current
  if (lhs->is_current != rhs->is_current) {
    return false;
  }
  // autocontinue
  if (lhs->autocontinue != rhs->autocontinue) {
    return false;
  }
  // param1
  if (lhs->param1 != rhs->param1) {
    return false;
  }
  // param2
  if (lhs->param2 != rhs->param2) {
    return false;
  }
  // param3
  if (lhs->param3 != rhs->param3) {
    return false;
  }
  // param4
  if (lhs->param4 != rhs->param4) {
    return false;
  }
  // x_lat
  if (lhs->x_lat != rhs->x_lat) {
    return false;
  }
  // y_long
  if (lhs->y_long != rhs->y_long) {
    return false;
  }
  // z_alt
  if (lhs->z_alt != rhs->z_alt) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__Waypoint__copy(
  const mavros_msgs__msg__Waypoint * input,
  mavros_msgs__msg__Waypoint * output)
{
  if (!input || !output) {
    return false;
  }
  // frame
  output->frame = input->frame;
  // command
  output->command = input->command;
  // is_current
  output->is_current = input->is_current;
  // autocontinue
  output->autocontinue = input->autocontinue;
  // param1
  output->param1 = input->param1;
  // param2
  output->param2 = input->param2;
  // param3
  output->param3 = input->param3;
  // param4
  output->param4 = input->param4;
  // x_lat
  output->x_lat = input->x_lat;
  // y_long
  output->y_long = input->y_long;
  // z_alt
  output->z_alt = input->z_alt;
  return true;
}

mavros_msgs__msg__Waypoint *
mavros_msgs__msg__Waypoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Waypoint * msg = (mavros_msgs__msg__Waypoint *)allocator.allocate(sizeof(mavros_msgs__msg__Waypoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__Waypoint));
  bool success = mavros_msgs__msg__Waypoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__Waypoint__destroy(mavros_msgs__msg__Waypoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__Waypoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__Waypoint__Sequence__init(mavros_msgs__msg__Waypoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Waypoint * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__Waypoint *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__Waypoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__Waypoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__Waypoint__fini(&data[i - 1]);
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
mavros_msgs__msg__Waypoint__Sequence__fini(mavros_msgs__msg__Waypoint__Sequence * array)
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
      mavros_msgs__msg__Waypoint__fini(&array->data[i]);
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

mavros_msgs__msg__Waypoint__Sequence *
mavros_msgs__msg__Waypoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Waypoint__Sequence * array = (mavros_msgs__msg__Waypoint__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__Waypoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__Waypoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__Waypoint__Sequence__destroy(mavros_msgs__msg__Waypoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__Waypoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__Waypoint__Sequence__are_equal(const mavros_msgs__msg__Waypoint__Sequence * lhs, const mavros_msgs__msg__Waypoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__Waypoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Waypoint__Sequence__copy(
  const mavros_msgs__msg__Waypoint__Sequence * input,
  mavros_msgs__msg__Waypoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__Waypoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__Waypoint * data =
      (mavros_msgs__msg__Waypoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__Waypoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__Waypoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__Waypoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
