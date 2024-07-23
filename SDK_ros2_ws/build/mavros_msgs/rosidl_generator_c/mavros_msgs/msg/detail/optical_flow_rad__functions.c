// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/OpticalFlowRad.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/optical_flow_rad__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__OpticalFlowRad__init(mavros_msgs__msg__OpticalFlowRad * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__OpticalFlowRad__fini(msg);
    return false;
  }
  // integration_time_us
  // integrated_x
  // integrated_y
  // integrated_xgyro
  // integrated_ygyro
  // integrated_zgyro
  // temperature
  // quality
  // time_delta_distance_us
  // distance
  return true;
}

void
mavros_msgs__msg__OpticalFlowRad__fini(mavros_msgs__msg__OpticalFlowRad * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // integration_time_us
  // integrated_x
  // integrated_y
  // integrated_xgyro
  // integrated_ygyro
  // integrated_zgyro
  // temperature
  // quality
  // time_delta_distance_us
  // distance
}

bool
mavros_msgs__msg__OpticalFlowRad__are_equal(const mavros_msgs__msg__OpticalFlowRad * lhs, const mavros_msgs__msg__OpticalFlowRad * rhs)
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
  // integration_time_us
  if (lhs->integration_time_us != rhs->integration_time_us) {
    return false;
  }
  // integrated_x
  if (lhs->integrated_x != rhs->integrated_x) {
    return false;
  }
  // integrated_y
  if (lhs->integrated_y != rhs->integrated_y) {
    return false;
  }
  // integrated_xgyro
  if (lhs->integrated_xgyro != rhs->integrated_xgyro) {
    return false;
  }
  // integrated_ygyro
  if (lhs->integrated_ygyro != rhs->integrated_ygyro) {
    return false;
  }
  // integrated_zgyro
  if (lhs->integrated_zgyro != rhs->integrated_zgyro) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // quality
  if (lhs->quality != rhs->quality) {
    return false;
  }
  // time_delta_distance_us
  if (lhs->time_delta_distance_us != rhs->time_delta_distance_us) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__OpticalFlowRad__copy(
  const mavros_msgs__msg__OpticalFlowRad * input,
  mavros_msgs__msg__OpticalFlowRad * output)
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
  // integration_time_us
  output->integration_time_us = input->integration_time_us;
  // integrated_x
  output->integrated_x = input->integrated_x;
  // integrated_y
  output->integrated_y = input->integrated_y;
  // integrated_xgyro
  output->integrated_xgyro = input->integrated_xgyro;
  // integrated_ygyro
  output->integrated_ygyro = input->integrated_ygyro;
  // integrated_zgyro
  output->integrated_zgyro = input->integrated_zgyro;
  // temperature
  output->temperature = input->temperature;
  // quality
  output->quality = input->quality;
  // time_delta_distance_us
  output->time_delta_distance_us = input->time_delta_distance_us;
  // distance
  output->distance = input->distance;
  return true;
}

mavros_msgs__msg__OpticalFlowRad *
mavros_msgs__msg__OpticalFlowRad__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpticalFlowRad * msg = (mavros_msgs__msg__OpticalFlowRad *)allocator.allocate(sizeof(mavros_msgs__msg__OpticalFlowRad), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__OpticalFlowRad));
  bool success = mavros_msgs__msg__OpticalFlowRad__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__OpticalFlowRad__destroy(mavros_msgs__msg__OpticalFlowRad * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__OpticalFlowRad__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__OpticalFlowRad__Sequence__init(mavros_msgs__msg__OpticalFlowRad__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpticalFlowRad * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__OpticalFlowRad *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__OpticalFlowRad), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__OpticalFlowRad__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__OpticalFlowRad__fini(&data[i - 1]);
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
mavros_msgs__msg__OpticalFlowRad__Sequence__fini(mavros_msgs__msg__OpticalFlowRad__Sequence * array)
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
      mavros_msgs__msg__OpticalFlowRad__fini(&array->data[i]);
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

mavros_msgs__msg__OpticalFlowRad__Sequence *
mavros_msgs__msg__OpticalFlowRad__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpticalFlowRad__Sequence * array = (mavros_msgs__msg__OpticalFlowRad__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__OpticalFlowRad__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__OpticalFlowRad__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__OpticalFlowRad__Sequence__destroy(mavros_msgs__msg__OpticalFlowRad__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__OpticalFlowRad__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__OpticalFlowRad__Sequence__are_equal(const mavros_msgs__msg__OpticalFlowRad__Sequence * lhs, const mavros_msgs__msg__OpticalFlowRad__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__OpticalFlowRad__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__OpticalFlowRad__Sequence__copy(
  const mavros_msgs__msg__OpticalFlowRad__Sequence * input,
  mavros_msgs__msg__OpticalFlowRad__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__OpticalFlowRad);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__OpticalFlowRad * data =
      (mavros_msgs__msg__OpticalFlowRad *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__OpticalFlowRad__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__OpticalFlowRad__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__OpticalFlowRad__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
