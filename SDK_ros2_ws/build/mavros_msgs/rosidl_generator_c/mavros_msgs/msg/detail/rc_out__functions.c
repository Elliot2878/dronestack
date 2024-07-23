// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/RCOut.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/rc_out__functions.h"

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
mavros_msgs__msg__RCOut__init(mavros_msgs__msg__RCOut * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__RCOut__fini(msg);
    return false;
  }
  // channels
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->channels, 0)) {
    mavros_msgs__msg__RCOut__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__RCOut__fini(mavros_msgs__msg__RCOut * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // channels
  rosidl_runtime_c__uint16__Sequence__fini(&msg->channels);
}

bool
mavros_msgs__msg__RCOut__are_equal(const mavros_msgs__msg__RCOut * lhs, const mavros_msgs__msg__RCOut * rhs)
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
  // channels
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->channels), &(rhs->channels)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__RCOut__copy(
  const mavros_msgs__msg__RCOut * input,
  mavros_msgs__msg__RCOut * output)
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
  // channels
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->channels), &(output->channels)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__RCOut *
mavros_msgs__msg__RCOut__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__RCOut * msg = (mavros_msgs__msg__RCOut *)allocator.allocate(sizeof(mavros_msgs__msg__RCOut), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__RCOut));
  bool success = mavros_msgs__msg__RCOut__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__RCOut__destroy(mavros_msgs__msg__RCOut * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__RCOut__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__RCOut__Sequence__init(mavros_msgs__msg__RCOut__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__RCOut * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__RCOut *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__RCOut), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__RCOut__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__RCOut__fini(&data[i - 1]);
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
mavros_msgs__msg__RCOut__Sequence__fini(mavros_msgs__msg__RCOut__Sequence * array)
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
      mavros_msgs__msg__RCOut__fini(&array->data[i]);
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

mavros_msgs__msg__RCOut__Sequence *
mavros_msgs__msg__RCOut__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__RCOut__Sequence * array = (mavros_msgs__msg__RCOut__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__RCOut__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__RCOut__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__RCOut__Sequence__destroy(mavros_msgs__msg__RCOut__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__RCOut__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__RCOut__Sequence__are_equal(const mavros_msgs__msg__RCOut__Sequence * lhs, const mavros_msgs__msg__RCOut__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__RCOut__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__RCOut__Sequence__copy(
  const mavros_msgs__msg__RCOut__Sequence * input,
  mavros_msgs__msg__RCOut__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__RCOut);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__RCOut * data =
      (mavros_msgs__msg__RCOut *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__RCOut__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__RCOut__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__RCOut__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
