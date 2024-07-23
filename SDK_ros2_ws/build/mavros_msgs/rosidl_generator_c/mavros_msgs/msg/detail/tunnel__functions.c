// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/Tunnel.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/tunnel__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mavros_msgs__msg__Tunnel__init(mavros_msgs__msg__Tunnel * msg)
{
  if (!msg) {
    return false;
  }
  // target_system
  // target_component
  // payload_type
  // payload_length
  // payload
  return true;
}

void
mavros_msgs__msg__Tunnel__fini(mavros_msgs__msg__Tunnel * msg)
{
  if (!msg) {
    return;
  }
  // target_system
  // target_component
  // payload_type
  // payload_length
  // payload
}

bool
mavros_msgs__msg__Tunnel__are_equal(const mavros_msgs__msg__Tunnel * lhs, const mavros_msgs__msg__Tunnel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_system
  if (lhs->target_system != rhs->target_system) {
    return false;
  }
  // target_component
  if (lhs->target_component != rhs->target_component) {
    return false;
  }
  // payload_type
  if (lhs->payload_type != rhs->payload_type) {
    return false;
  }
  // payload_length
  if (lhs->payload_length != rhs->payload_length) {
    return false;
  }
  // payload
  for (size_t i = 0; i < 128; ++i) {
    if (lhs->payload[i] != rhs->payload[i]) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Tunnel__copy(
  const mavros_msgs__msg__Tunnel * input,
  mavros_msgs__msg__Tunnel * output)
{
  if (!input || !output) {
    return false;
  }
  // target_system
  output->target_system = input->target_system;
  // target_component
  output->target_component = input->target_component;
  // payload_type
  output->payload_type = input->payload_type;
  // payload_length
  output->payload_length = input->payload_length;
  // payload
  for (size_t i = 0; i < 128; ++i) {
    output->payload[i] = input->payload[i];
  }
  return true;
}

mavros_msgs__msg__Tunnel *
mavros_msgs__msg__Tunnel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Tunnel * msg = (mavros_msgs__msg__Tunnel *)allocator.allocate(sizeof(mavros_msgs__msg__Tunnel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__Tunnel));
  bool success = mavros_msgs__msg__Tunnel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__Tunnel__destroy(mavros_msgs__msg__Tunnel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__Tunnel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__Tunnel__Sequence__init(mavros_msgs__msg__Tunnel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Tunnel * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__Tunnel *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__Tunnel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__Tunnel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__Tunnel__fini(&data[i - 1]);
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
mavros_msgs__msg__Tunnel__Sequence__fini(mavros_msgs__msg__Tunnel__Sequence * array)
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
      mavros_msgs__msg__Tunnel__fini(&array->data[i]);
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

mavros_msgs__msg__Tunnel__Sequence *
mavros_msgs__msg__Tunnel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Tunnel__Sequence * array = (mavros_msgs__msg__Tunnel__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__Tunnel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__Tunnel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__Tunnel__Sequence__destroy(mavros_msgs__msg__Tunnel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__Tunnel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__Tunnel__Sequence__are_equal(const mavros_msgs__msg__Tunnel__Sequence * lhs, const mavros_msgs__msg__Tunnel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__Tunnel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Tunnel__Sequence__copy(
  const mavros_msgs__msg__Tunnel__Sequence * input,
  mavros_msgs__msg__Tunnel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__Tunnel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__Tunnel * data =
      (mavros_msgs__msg__Tunnel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__Tunnel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__Tunnel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__Tunnel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
