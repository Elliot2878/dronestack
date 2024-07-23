// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/StatusText.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/status_text__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `text`
#include "rosidl_runtime_c/string_functions.h"

bool
mavros_msgs__msg__StatusText__init(mavros_msgs__msg__StatusText * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__StatusText__fini(msg);
    return false;
  }
  // severity
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    mavros_msgs__msg__StatusText__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__StatusText__fini(mavros_msgs__msg__StatusText * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // severity
  // text
  rosidl_runtime_c__String__fini(&msg->text);
}

bool
mavros_msgs__msg__StatusText__are_equal(const mavros_msgs__msg__StatusText * lhs, const mavros_msgs__msg__StatusText * rhs)
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
  // severity
  if (lhs->severity != rhs->severity) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__StatusText__copy(
  const mavros_msgs__msg__StatusText * input,
  mavros_msgs__msg__StatusText * output)
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
  // severity
  output->severity = input->severity;
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__StatusText *
mavros_msgs__msg__StatusText__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__StatusText * msg = (mavros_msgs__msg__StatusText *)allocator.allocate(sizeof(mavros_msgs__msg__StatusText), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__StatusText));
  bool success = mavros_msgs__msg__StatusText__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__StatusText__destroy(mavros_msgs__msg__StatusText * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__StatusText__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__StatusText__Sequence__init(mavros_msgs__msg__StatusText__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__StatusText * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__StatusText *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__StatusText), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__StatusText__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__StatusText__fini(&data[i - 1]);
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
mavros_msgs__msg__StatusText__Sequence__fini(mavros_msgs__msg__StatusText__Sequence * array)
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
      mavros_msgs__msg__StatusText__fini(&array->data[i]);
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

mavros_msgs__msg__StatusText__Sequence *
mavros_msgs__msg__StatusText__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__StatusText__Sequence * array = (mavros_msgs__msg__StatusText__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__StatusText__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__StatusText__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__StatusText__Sequence__destroy(mavros_msgs__msg__StatusText__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__StatusText__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__StatusText__Sequence__are_equal(const mavros_msgs__msg__StatusText__Sequence * lhs, const mavros_msgs__msg__StatusText__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__StatusText__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__StatusText__Sequence__copy(
  const mavros_msgs__msg__StatusText__Sequence * input,
  mavros_msgs__msg__StatusText__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__StatusText);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__StatusText * data =
      (mavros_msgs__msg__StatusText *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__StatusText__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__StatusText__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__StatusText__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
