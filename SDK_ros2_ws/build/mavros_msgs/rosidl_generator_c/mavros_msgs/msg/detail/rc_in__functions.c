// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/RCIn.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/rc_in__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `channels`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mavros_msgs__msg__RCIn__init(mavros_msgs__msg__RCIn * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__RCIn__fini(msg);
    return false;
  }
  // rssi
  // channels
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->channels, 0)) {
    mavros_msgs__msg__RCIn__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__RCIn__fini(mavros_msgs__msg__RCIn * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rssi
  // channels
  rosidl_runtime_c__uint16__Sequence__fini(&msg->channels);
}

bool
mavros_msgs__msg__RCIn__are_equal(const mavros_msgs__msg__RCIn * lhs, const mavros_msgs__msg__RCIn * rhs)
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
  // rssi
  if (lhs->rssi != rhs->rssi) {
    return false;
  }
  // channels
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->channels), &(rhs->channels)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__RCIn__copy(
  const mavros_msgs__msg__RCIn * input,
  mavros_msgs__msg__RCIn * output)
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
  // rssi
  output->rssi = input->rssi;
  // channels
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->channels), &(output->channels)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__RCIn *
mavros_msgs__msg__RCIn__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__RCIn * msg = (mavros_msgs__msg__RCIn *)allocator.allocate(sizeof(mavros_msgs__msg__RCIn), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__RCIn));
  bool success = mavros_msgs__msg__RCIn__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__RCIn__destroy(mavros_msgs__msg__RCIn * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__RCIn__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__RCIn__Sequence__init(mavros_msgs__msg__RCIn__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__RCIn * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__RCIn *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__RCIn), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__RCIn__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__RCIn__fini(&data[i - 1]);
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
mavros_msgs__msg__RCIn__Sequence__fini(mavros_msgs__msg__RCIn__Sequence * array)
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
      mavros_msgs__msg__RCIn__fini(&array->data[i]);
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

mavros_msgs__msg__RCIn__Sequence *
mavros_msgs__msg__RCIn__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__RCIn__Sequence * array = (mavros_msgs__msg__RCIn__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__RCIn__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__RCIn__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__RCIn__Sequence__destroy(mavros_msgs__msg__RCIn__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__RCIn__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__RCIn__Sequence__are_equal(const mavros_msgs__msg__RCIn__Sequence * lhs, const mavros_msgs__msg__RCIn__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__RCIn__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__RCIn__Sequence__copy(
  const mavros_msgs__msg__RCIn__Sequence * input,
  mavros_msgs__msg__RCIn__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__RCIn);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__RCIn * data =
      (mavros_msgs__msg__RCIn *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__RCIn__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__RCIn__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__RCIn__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
