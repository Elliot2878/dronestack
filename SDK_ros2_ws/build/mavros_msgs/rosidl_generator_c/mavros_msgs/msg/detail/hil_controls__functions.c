// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/HilControls.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/hil_controls__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__HilControls__init(mavros_msgs__msg__HilControls * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__HilControls__fini(msg);
    return false;
  }
  // roll_ailerons
  // pitch_elevator
  // yaw_rudder
  // throttle
  // aux1
  // aux2
  // aux3
  // aux4
  // mode
  // nav_mode
  return true;
}

void
mavros_msgs__msg__HilControls__fini(mavros_msgs__msg__HilControls * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // roll_ailerons
  // pitch_elevator
  // yaw_rudder
  // throttle
  // aux1
  // aux2
  // aux3
  // aux4
  // mode
  // nav_mode
}

bool
mavros_msgs__msg__HilControls__are_equal(const mavros_msgs__msg__HilControls * lhs, const mavros_msgs__msg__HilControls * rhs)
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
  // roll_ailerons
  if (lhs->roll_ailerons != rhs->roll_ailerons) {
    return false;
  }
  // pitch_elevator
  if (lhs->pitch_elevator != rhs->pitch_elevator) {
    return false;
  }
  // yaw_rudder
  if (lhs->yaw_rudder != rhs->yaw_rudder) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // aux1
  if (lhs->aux1 != rhs->aux1) {
    return false;
  }
  // aux2
  if (lhs->aux2 != rhs->aux2) {
    return false;
  }
  // aux3
  if (lhs->aux3 != rhs->aux3) {
    return false;
  }
  // aux4
  if (lhs->aux4 != rhs->aux4) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // nav_mode
  if (lhs->nav_mode != rhs->nav_mode) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__HilControls__copy(
  const mavros_msgs__msg__HilControls * input,
  mavros_msgs__msg__HilControls * output)
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
  // roll_ailerons
  output->roll_ailerons = input->roll_ailerons;
  // pitch_elevator
  output->pitch_elevator = input->pitch_elevator;
  // yaw_rudder
  output->yaw_rudder = input->yaw_rudder;
  // throttle
  output->throttle = input->throttle;
  // aux1
  output->aux1 = input->aux1;
  // aux2
  output->aux2 = input->aux2;
  // aux3
  output->aux3 = input->aux3;
  // aux4
  output->aux4 = input->aux4;
  // mode
  output->mode = input->mode;
  // nav_mode
  output->nav_mode = input->nav_mode;
  return true;
}

mavros_msgs__msg__HilControls *
mavros_msgs__msg__HilControls__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilControls * msg = (mavros_msgs__msg__HilControls *)allocator.allocate(sizeof(mavros_msgs__msg__HilControls), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__HilControls));
  bool success = mavros_msgs__msg__HilControls__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__HilControls__destroy(mavros_msgs__msg__HilControls * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__HilControls__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__HilControls__Sequence__init(mavros_msgs__msg__HilControls__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilControls * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__HilControls *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__HilControls), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__HilControls__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__HilControls__fini(&data[i - 1]);
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
mavros_msgs__msg__HilControls__Sequence__fini(mavros_msgs__msg__HilControls__Sequence * array)
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
      mavros_msgs__msg__HilControls__fini(&array->data[i]);
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

mavros_msgs__msg__HilControls__Sequence *
mavros_msgs__msg__HilControls__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilControls__Sequence * array = (mavros_msgs__msg__HilControls__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__HilControls__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__HilControls__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__HilControls__Sequence__destroy(mavros_msgs__msg__HilControls__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__HilControls__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__HilControls__Sequence__are_equal(const mavros_msgs__msg__HilControls__Sequence * lhs, const mavros_msgs__msg__HilControls__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__HilControls__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__HilControls__Sequence__copy(
  const mavros_msgs__msg__HilControls__Sequence * input,
  mavros_msgs__msg__HilControls__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__HilControls);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__HilControls * data =
      (mavros_msgs__msg__HilControls *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__HilControls__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__HilControls__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__HilControls__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
