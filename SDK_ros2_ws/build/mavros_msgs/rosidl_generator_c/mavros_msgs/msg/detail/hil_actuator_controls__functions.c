// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/HilActuatorControls.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/hil_actuator_controls__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__HilActuatorControls__init(mavros_msgs__msg__HilActuatorControls * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__HilActuatorControls__fini(msg);
    return false;
  }
  // controls
  // mode
  // flags
  return true;
}

void
mavros_msgs__msg__HilActuatorControls__fini(mavros_msgs__msg__HilActuatorControls * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // controls
  // mode
  // flags
}

bool
mavros_msgs__msg__HilActuatorControls__are_equal(const mavros_msgs__msg__HilActuatorControls * lhs, const mavros_msgs__msg__HilActuatorControls * rhs)
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
  // controls
  for (size_t i = 0; i < 16; ++i) {
    if (lhs->controls[i] != rhs->controls[i]) {
      return false;
    }
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // flags
  if (lhs->flags != rhs->flags) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__HilActuatorControls__copy(
  const mavros_msgs__msg__HilActuatorControls * input,
  mavros_msgs__msg__HilActuatorControls * output)
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
  // controls
  for (size_t i = 0; i < 16; ++i) {
    output->controls[i] = input->controls[i];
  }
  // mode
  output->mode = input->mode;
  // flags
  output->flags = input->flags;
  return true;
}

mavros_msgs__msg__HilActuatorControls *
mavros_msgs__msg__HilActuatorControls__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilActuatorControls * msg = (mavros_msgs__msg__HilActuatorControls *)allocator.allocate(sizeof(mavros_msgs__msg__HilActuatorControls), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__HilActuatorControls));
  bool success = mavros_msgs__msg__HilActuatorControls__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__HilActuatorControls__destroy(mavros_msgs__msg__HilActuatorControls * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__HilActuatorControls__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__HilActuatorControls__Sequence__init(mavros_msgs__msg__HilActuatorControls__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilActuatorControls * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__HilActuatorControls *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__HilActuatorControls), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__HilActuatorControls__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__HilActuatorControls__fini(&data[i - 1]);
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
mavros_msgs__msg__HilActuatorControls__Sequence__fini(mavros_msgs__msg__HilActuatorControls__Sequence * array)
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
      mavros_msgs__msg__HilActuatorControls__fini(&array->data[i]);
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

mavros_msgs__msg__HilActuatorControls__Sequence *
mavros_msgs__msg__HilActuatorControls__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilActuatorControls__Sequence * array = (mavros_msgs__msg__HilActuatorControls__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__HilActuatorControls__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__HilActuatorControls__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__HilActuatorControls__Sequence__destroy(mavros_msgs__msg__HilActuatorControls__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__HilActuatorControls__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__HilActuatorControls__Sequence__are_equal(const mavros_msgs__msg__HilActuatorControls__Sequence * lhs, const mavros_msgs__msg__HilActuatorControls__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__HilActuatorControls__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__HilActuatorControls__Sequence__copy(
  const mavros_msgs__msg__HilActuatorControls__Sequence * input,
  mavros_msgs__msg__HilActuatorControls__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__HilActuatorControls);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__HilActuatorControls * data =
      (mavros_msgs__msg__HilActuatorControls *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__HilActuatorControls__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__HilActuatorControls__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__HilActuatorControls__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
