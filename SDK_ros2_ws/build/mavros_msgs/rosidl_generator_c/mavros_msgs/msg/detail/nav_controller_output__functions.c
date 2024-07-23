// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/NavControllerOutput.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/nav_controller_output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__NavControllerOutput__init(mavros_msgs__msg__NavControllerOutput * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__NavControllerOutput__fini(msg);
    return false;
  }
  // nav_roll
  // nav_pitch
  // nav_bearing
  // target_bearing
  // wp_dist
  // alt_error
  // aspd_error
  // xtrack_error
  return true;
}

void
mavros_msgs__msg__NavControllerOutput__fini(mavros_msgs__msg__NavControllerOutput * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // nav_roll
  // nav_pitch
  // nav_bearing
  // target_bearing
  // wp_dist
  // alt_error
  // aspd_error
  // xtrack_error
}

bool
mavros_msgs__msg__NavControllerOutput__are_equal(const mavros_msgs__msg__NavControllerOutput * lhs, const mavros_msgs__msg__NavControllerOutput * rhs)
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
  // nav_roll
  if (lhs->nav_roll != rhs->nav_roll) {
    return false;
  }
  // nav_pitch
  if (lhs->nav_pitch != rhs->nav_pitch) {
    return false;
  }
  // nav_bearing
  if (lhs->nav_bearing != rhs->nav_bearing) {
    return false;
  }
  // target_bearing
  if (lhs->target_bearing != rhs->target_bearing) {
    return false;
  }
  // wp_dist
  if (lhs->wp_dist != rhs->wp_dist) {
    return false;
  }
  // alt_error
  if (lhs->alt_error != rhs->alt_error) {
    return false;
  }
  // aspd_error
  if (lhs->aspd_error != rhs->aspd_error) {
    return false;
  }
  // xtrack_error
  if (lhs->xtrack_error != rhs->xtrack_error) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__NavControllerOutput__copy(
  const mavros_msgs__msg__NavControllerOutput * input,
  mavros_msgs__msg__NavControllerOutput * output)
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
  // nav_roll
  output->nav_roll = input->nav_roll;
  // nav_pitch
  output->nav_pitch = input->nav_pitch;
  // nav_bearing
  output->nav_bearing = input->nav_bearing;
  // target_bearing
  output->target_bearing = input->target_bearing;
  // wp_dist
  output->wp_dist = input->wp_dist;
  // alt_error
  output->alt_error = input->alt_error;
  // aspd_error
  output->aspd_error = input->aspd_error;
  // xtrack_error
  output->xtrack_error = input->xtrack_error;
  return true;
}

mavros_msgs__msg__NavControllerOutput *
mavros_msgs__msg__NavControllerOutput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__NavControllerOutput * msg = (mavros_msgs__msg__NavControllerOutput *)allocator.allocate(sizeof(mavros_msgs__msg__NavControllerOutput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__NavControllerOutput));
  bool success = mavros_msgs__msg__NavControllerOutput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__NavControllerOutput__destroy(mavros_msgs__msg__NavControllerOutput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__NavControllerOutput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__NavControllerOutput__Sequence__init(mavros_msgs__msg__NavControllerOutput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__NavControllerOutput * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__NavControllerOutput *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__NavControllerOutput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__NavControllerOutput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__NavControllerOutput__fini(&data[i - 1]);
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
mavros_msgs__msg__NavControllerOutput__Sequence__fini(mavros_msgs__msg__NavControllerOutput__Sequence * array)
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
      mavros_msgs__msg__NavControllerOutput__fini(&array->data[i]);
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

mavros_msgs__msg__NavControllerOutput__Sequence *
mavros_msgs__msg__NavControllerOutput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__NavControllerOutput__Sequence * array = (mavros_msgs__msg__NavControllerOutput__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__NavControllerOutput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__NavControllerOutput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__NavControllerOutput__Sequence__destroy(mavros_msgs__msg__NavControllerOutput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__NavControllerOutput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__NavControllerOutput__Sequence__are_equal(const mavros_msgs__msg__NavControllerOutput__Sequence * lhs, const mavros_msgs__msg__NavControllerOutput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__NavControllerOutput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__NavControllerOutput__Sequence__copy(
  const mavros_msgs__msg__NavControllerOutput__Sequence * input,
  mavros_msgs__msg__NavControllerOutput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__NavControllerOutput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__NavControllerOutput * data =
      (mavros_msgs__msg__NavControllerOutput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__NavControllerOutput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__NavControllerOutput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__NavControllerOutput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
