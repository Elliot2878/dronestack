// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/ESCTelemetryItem.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/esc_telemetry_item__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__ESCTelemetryItem__init(mavros_msgs__msg__ESCTelemetryItem * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__ESCTelemetryItem__fini(msg);
    return false;
  }
  // temperature
  // voltage
  // current
  // totalcurrent
  // rpm
  // count
  return true;
}

void
mavros_msgs__msg__ESCTelemetryItem__fini(mavros_msgs__msg__ESCTelemetryItem * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // temperature
  // voltage
  // current
  // totalcurrent
  // rpm
  // count
}

bool
mavros_msgs__msg__ESCTelemetryItem__are_equal(const mavros_msgs__msg__ESCTelemetryItem * lhs, const mavros_msgs__msg__ESCTelemetryItem * rhs)
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
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // totalcurrent
  if (lhs->totalcurrent != rhs->totalcurrent) {
    return false;
  }
  // rpm
  if (lhs->rpm != rhs->rpm) {
    return false;
  }
  // count
  if (lhs->count != rhs->count) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__ESCTelemetryItem__copy(
  const mavros_msgs__msg__ESCTelemetryItem * input,
  mavros_msgs__msg__ESCTelemetryItem * output)
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
  // temperature
  output->temperature = input->temperature;
  // voltage
  output->voltage = input->voltage;
  // current
  output->current = input->current;
  // totalcurrent
  output->totalcurrent = input->totalcurrent;
  // rpm
  output->rpm = input->rpm;
  // count
  output->count = input->count;
  return true;
}

mavros_msgs__msg__ESCTelemetryItem *
mavros_msgs__msg__ESCTelemetryItem__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ESCTelemetryItem * msg = (mavros_msgs__msg__ESCTelemetryItem *)allocator.allocate(sizeof(mavros_msgs__msg__ESCTelemetryItem), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__ESCTelemetryItem));
  bool success = mavros_msgs__msg__ESCTelemetryItem__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__ESCTelemetryItem__destroy(mavros_msgs__msg__ESCTelemetryItem * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__ESCTelemetryItem__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__ESCTelemetryItem__Sequence__init(mavros_msgs__msg__ESCTelemetryItem__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ESCTelemetryItem * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__ESCTelemetryItem *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__ESCTelemetryItem), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__ESCTelemetryItem__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__ESCTelemetryItem__fini(&data[i - 1]);
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
mavros_msgs__msg__ESCTelemetryItem__Sequence__fini(mavros_msgs__msg__ESCTelemetryItem__Sequence * array)
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
      mavros_msgs__msg__ESCTelemetryItem__fini(&array->data[i]);
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

mavros_msgs__msg__ESCTelemetryItem__Sequence *
mavros_msgs__msg__ESCTelemetryItem__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ESCTelemetryItem__Sequence * array = (mavros_msgs__msg__ESCTelemetryItem__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__ESCTelemetryItem__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__ESCTelemetryItem__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__ESCTelemetryItem__Sequence__destroy(mavros_msgs__msg__ESCTelemetryItem__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__ESCTelemetryItem__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__ESCTelemetryItem__Sequence__are_equal(const mavros_msgs__msg__ESCTelemetryItem__Sequence * lhs, const mavros_msgs__msg__ESCTelemetryItem__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__ESCTelemetryItem__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__ESCTelemetryItem__Sequence__copy(
  const mavros_msgs__msg__ESCTelemetryItem__Sequence * input,
  mavros_msgs__msg__ESCTelemetryItem__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__ESCTelemetryItem);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__ESCTelemetryItem * data =
      (mavros_msgs__msg__ESCTelemetryItem *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__ESCTelemetryItem__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__ESCTelemetryItem__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__ESCTelemetryItem__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
