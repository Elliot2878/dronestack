// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/TerrainReport.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/terrain_report__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__TerrainReport__init(mavros_msgs__msg__TerrainReport * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__TerrainReport__fini(msg);
    return false;
  }
  // latitude
  // longitude
  // spacing
  // terrain_height
  // current_height
  // pending
  // loaded
  return true;
}

void
mavros_msgs__msg__TerrainReport__fini(mavros_msgs__msg__TerrainReport * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // latitude
  // longitude
  // spacing
  // terrain_height
  // current_height
  // pending
  // loaded
}

bool
mavros_msgs__msg__TerrainReport__are_equal(const mavros_msgs__msg__TerrainReport * lhs, const mavros_msgs__msg__TerrainReport * rhs)
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
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // spacing
  if (lhs->spacing != rhs->spacing) {
    return false;
  }
  // terrain_height
  if (lhs->terrain_height != rhs->terrain_height) {
    return false;
  }
  // current_height
  if (lhs->current_height != rhs->current_height) {
    return false;
  }
  // pending
  if (lhs->pending != rhs->pending) {
    return false;
  }
  // loaded
  if (lhs->loaded != rhs->loaded) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__TerrainReport__copy(
  const mavros_msgs__msg__TerrainReport * input,
  mavros_msgs__msg__TerrainReport * output)
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
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // spacing
  output->spacing = input->spacing;
  // terrain_height
  output->terrain_height = input->terrain_height;
  // current_height
  output->current_height = input->current_height;
  // pending
  output->pending = input->pending;
  // loaded
  output->loaded = input->loaded;
  return true;
}

mavros_msgs__msg__TerrainReport *
mavros_msgs__msg__TerrainReport__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__TerrainReport * msg = (mavros_msgs__msg__TerrainReport *)allocator.allocate(sizeof(mavros_msgs__msg__TerrainReport), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__TerrainReport));
  bool success = mavros_msgs__msg__TerrainReport__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__TerrainReport__destroy(mavros_msgs__msg__TerrainReport * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__TerrainReport__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__TerrainReport__Sequence__init(mavros_msgs__msg__TerrainReport__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__TerrainReport * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__TerrainReport *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__TerrainReport), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__TerrainReport__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__TerrainReport__fini(&data[i - 1]);
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
mavros_msgs__msg__TerrainReport__Sequence__fini(mavros_msgs__msg__TerrainReport__Sequence * array)
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
      mavros_msgs__msg__TerrainReport__fini(&array->data[i]);
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

mavros_msgs__msg__TerrainReport__Sequence *
mavros_msgs__msg__TerrainReport__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__TerrainReport__Sequence * array = (mavros_msgs__msg__TerrainReport__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__TerrainReport__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__TerrainReport__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__TerrainReport__Sequence__destroy(mavros_msgs__msg__TerrainReport__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__TerrainReport__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__TerrainReport__Sequence__are_equal(const mavros_msgs__msg__TerrainReport__Sequence * lhs, const mavros_msgs__msg__TerrainReport__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__TerrainReport__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__TerrainReport__Sequence__copy(
  const mavros_msgs__msg__TerrainReport__Sequence * input,
  mavros_msgs__msg__TerrainReport__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__TerrainReport);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__TerrainReport * data =
      (mavros_msgs__msg__TerrainReport *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__TerrainReport__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__TerrainReport__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__TerrainReport__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
