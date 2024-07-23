// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/VfrHud.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/vfr_hud__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__VfrHud__init(mavros_msgs__msg__VfrHud * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__VfrHud__fini(msg);
    return false;
  }
  // airspeed
  // groundspeed
  // heading
  // throttle
  // altitude
  // climb
  return true;
}

void
mavros_msgs__msg__VfrHud__fini(mavros_msgs__msg__VfrHud * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // airspeed
  // groundspeed
  // heading
  // throttle
  // altitude
  // climb
}

bool
mavros_msgs__msg__VfrHud__are_equal(const mavros_msgs__msg__VfrHud * lhs, const mavros_msgs__msg__VfrHud * rhs)
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
  // airspeed
  if (lhs->airspeed != rhs->airspeed) {
    return false;
  }
  // groundspeed
  if (lhs->groundspeed != rhs->groundspeed) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // climb
  if (lhs->climb != rhs->climb) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__VfrHud__copy(
  const mavros_msgs__msg__VfrHud * input,
  mavros_msgs__msg__VfrHud * output)
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
  // airspeed
  output->airspeed = input->airspeed;
  // groundspeed
  output->groundspeed = input->groundspeed;
  // heading
  output->heading = input->heading;
  // throttle
  output->throttle = input->throttle;
  // altitude
  output->altitude = input->altitude;
  // climb
  output->climb = input->climb;
  return true;
}

mavros_msgs__msg__VfrHud *
mavros_msgs__msg__VfrHud__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__VfrHud * msg = (mavros_msgs__msg__VfrHud *)allocator.allocate(sizeof(mavros_msgs__msg__VfrHud), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__VfrHud));
  bool success = mavros_msgs__msg__VfrHud__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__VfrHud__destroy(mavros_msgs__msg__VfrHud * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__VfrHud__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__VfrHud__Sequence__init(mavros_msgs__msg__VfrHud__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__VfrHud * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__VfrHud *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__VfrHud), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__VfrHud__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__VfrHud__fini(&data[i - 1]);
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
mavros_msgs__msg__VfrHud__Sequence__fini(mavros_msgs__msg__VfrHud__Sequence * array)
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
      mavros_msgs__msg__VfrHud__fini(&array->data[i]);
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

mavros_msgs__msg__VfrHud__Sequence *
mavros_msgs__msg__VfrHud__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__VfrHud__Sequence * array = (mavros_msgs__msg__VfrHud__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__VfrHud__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__VfrHud__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__VfrHud__Sequence__destroy(mavros_msgs__msg__VfrHud__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__VfrHud__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__VfrHud__Sequence__are_equal(const mavros_msgs__msg__VfrHud__Sequence * lhs, const mavros_msgs__msg__VfrHud__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__VfrHud__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__VfrHud__Sequence__copy(
  const mavros_msgs__msg__VfrHud__Sequence * input,
  mavros_msgs__msg__VfrHud__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__VfrHud);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__VfrHud * data =
      (mavros_msgs__msg__VfrHud *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__VfrHud__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__VfrHud__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__VfrHud__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
