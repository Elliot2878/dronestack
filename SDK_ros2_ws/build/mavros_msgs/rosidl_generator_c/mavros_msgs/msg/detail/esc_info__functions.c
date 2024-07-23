// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/ESCInfo.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/esc_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `esc_info`
#include "mavros_msgs/msg/detail/esc_info_item__functions.h"

bool
mavros_msgs__msg__ESCInfo__init(mavros_msgs__msg__ESCInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__ESCInfo__fini(msg);
    return false;
  }
  // counter
  // count
  // connection_type
  // info
  // esc_info
  if (!mavros_msgs__msg__ESCInfoItem__Sequence__init(&msg->esc_info, 0)) {
    mavros_msgs__msg__ESCInfo__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__ESCInfo__fini(mavros_msgs__msg__ESCInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // counter
  // count
  // connection_type
  // info
  // esc_info
  mavros_msgs__msg__ESCInfoItem__Sequence__fini(&msg->esc_info);
}

bool
mavros_msgs__msg__ESCInfo__are_equal(const mavros_msgs__msg__ESCInfo * lhs, const mavros_msgs__msg__ESCInfo * rhs)
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
  // counter
  if (lhs->counter != rhs->counter) {
    return false;
  }
  // count
  if (lhs->count != rhs->count) {
    return false;
  }
  // connection_type
  if (lhs->connection_type != rhs->connection_type) {
    return false;
  }
  // info
  if (lhs->info != rhs->info) {
    return false;
  }
  // esc_info
  if (!mavros_msgs__msg__ESCInfoItem__Sequence__are_equal(
      &(lhs->esc_info), &(rhs->esc_info)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__ESCInfo__copy(
  const mavros_msgs__msg__ESCInfo * input,
  mavros_msgs__msg__ESCInfo * output)
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
  // counter
  output->counter = input->counter;
  // count
  output->count = input->count;
  // connection_type
  output->connection_type = input->connection_type;
  // info
  output->info = input->info;
  // esc_info
  if (!mavros_msgs__msg__ESCInfoItem__Sequence__copy(
      &(input->esc_info), &(output->esc_info)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__ESCInfo *
mavros_msgs__msg__ESCInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ESCInfo * msg = (mavros_msgs__msg__ESCInfo *)allocator.allocate(sizeof(mavros_msgs__msg__ESCInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__ESCInfo));
  bool success = mavros_msgs__msg__ESCInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__ESCInfo__destroy(mavros_msgs__msg__ESCInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__ESCInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__ESCInfo__Sequence__init(mavros_msgs__msg__ESCInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ESCInfo * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__ESCInfo *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__ESCInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__ESCInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__ESCInfo__fini(&data[i - 1]);
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
mavros_msgs__msg__ESCInfo__Sequence__fini(mavros_msgs__msg__ESCInfo__Sequence * array)
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
      mavros_msgs__msg__ESCInfo__fini(&array->data[i]);
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

mavros_msgs__msg__ESCInfo__Sequence *
mavros_msgs__msg__ESCInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ESCInfo__Sequence * array = (mavros_msgs__msg__ESCInfo__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__ESCInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__ESCInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__ESCInfo__Sequence__destroy(mavros_msgs__msg__ESCInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__ESCInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__ESCInfo__Sequence__are_equal(const mavros_msgs__msg__ESCInfo__Sequence * lhs, const mavros_msgs__msg__ESCInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__ESCInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__ESCInfo__Sequence__copy(
  const mavros_msgs__msg__ESCInfo__Sequence * input,
  mavros_msgs__msg__ESCInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__ESCInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__ESCInfo * data =
      (mavros_msgs__msg__ESCInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__ESCInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__ESCInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__ESCInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
