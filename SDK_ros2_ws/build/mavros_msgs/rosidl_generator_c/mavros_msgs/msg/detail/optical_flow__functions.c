// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/OpticalFlow.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/optical_flow__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `flow`
// Member `flow_comp_m`
// Member `flow_rate`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mavros_msgs__msg__OpticalFlow__init(mavros_msgs__msg__OpticalFlow * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__OpticalFlow__fini(msg);
    return false;
  }
  // flow
  if (!geometry_msgs__msg__Vector3__init(&msg->flow)) {
    mavros_msgs__msg__OpticalFlow__fini(msg);
    return false;
  }
  // flow_comp_m
  if (!geometry_msgs__msg__Vector3__init(&msg->flow_comp_m)) {
    mavros_msgs__msg__OpticalFlow__fini(msg);
    return false;
  }
  // quality
  // ground_distance
  // flow_rate
  if (!geometry_msgs__msg__Vector3__init(&msg->flow_rate)) {
    mavros_msgs__msg__OpticalFlow__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__OpticalFlow__fini(mavros_msgs__msg__OpticalFlow * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // flow
  geometry_msgs__msg__Vector3__fini(&msg->flow);
  // flow_comp_m
  geometry_msgs__msg__Vector3__fini(&msg->flow_comp_m);
  // quality
  // ground_distance
  // flow_rate
  geometry_msgs__msg__Vector3__fini(&msg->flow_rate);
}

bool
mavros_msgs__msg__OpticalFlow__are_equal(const mavros_msgs__msg__OpticalFlow * lhs, const mavros_msgs__msg__OpticalFlow * rhs)
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
  // flow
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->flow), &(rhs->flow)))
  {
    return false;
  }
  // flow_comp_m
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->flow_comp_m), &(rhs->flow_comp_m)))
  {
    return false;
  }
  // quality
  if (lhs->quality != rhs->quality) {
    return false;
  }
  // ground_distance
  if (lhs->ground_distance != rhs->ground_distance) {
    return false;
  }
  // flow_rate
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->flow_rate), &(rhs->flow_rate)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__OpticalFlow__copy(
  const mavros_msgs__msg__OpticalFlow * input,
  mavros_msgs__msg__OpticalFlow * output)
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
  // flow
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->flow), &(output->flow)))
  {
    return false;
  }
  // flow_comp_m
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->flow_comp_m), &(output->flow_comp_m)))
  {
    return false;
  }
  // quality
  output->quality = input->quality;
  // ground_distance
  output->ground_distance = input->ground_distance;
  // flow_rate
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->flow_rate), &(output->flow_rate)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__OpticalFlow *
mavros_msgs__msg__OpticalFlow__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpticalFlow * msg = (mavros_msgs__msg__OpticalFlow *)allocator.allocate(sizeof(mavros_msgs__msg__OpticalFlow), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__OpticalFlow));
  bool success = mavros_msgs__msg__OpticalFlow__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__OpticalFlow__destroy(mavros_msgs__msg__OpticalFlow * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__OpticalFlow__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__OpticalFlow__Sequence__init(mavros_msgs__msg__OpticalFlow__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpticalFlow * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__OpticalFlow *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__OpticalFlow), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__OpticalFlow__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__OpticalFlow__fini(&data[i - 1]);
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
mavros_msgs__msg__OpticalFlow__Sequence__fini(mavros_msgs__msg__OpticalFlow__Sequence * array)
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
      mavros_msgs__msg__OpticalFlow__fini(&array->data[i]);
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

mavros_msgs__msg__OpticalFlow__Sequence *
mavros_msgs__msg__OpticalFlow__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OpticalFlow__Sequence * array = (mavros_msgs__msg__OpticalFlow__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__OpticalFlow__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__OpticalFlow__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__OpticalFlow__Sequence__destroy(mavros_msgs__msg__OpticalFlow__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__OpticalFlow__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__OpticalFlow__Sequence__are_equal(const mavros_msgs__msg__OpticalFlow__Sequence * lhs, const mavros_msgs__msg__OpticalFlow__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__OpticalFlow__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__OpticalFlow__Sequence__copy(
  const mavros_msgs__msg__OpticalFlow__Sequence * input,
  mavros_msgs__msg__OpticalFlow__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__OpticalFlow);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__OpticalFlow * data =
      (mavros_msgs__msg__OpticalFlow *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__OpticalFlow__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__OpticalFlow__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__OpticalFlow__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
