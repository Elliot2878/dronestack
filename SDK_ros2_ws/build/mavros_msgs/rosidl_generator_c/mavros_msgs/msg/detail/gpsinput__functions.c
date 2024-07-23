// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/GPSINPUT.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/gpsinput__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__GPSINPUT__init(mavros_msgs__msg__GPSINPUT * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__GPSINPUT__fini(msg);
    return false;
  }
  // fix_type
  // gps_id
  // ignore_flags
  // time_week_ms
  // time_week
  // lat
  // lon
  // alt
  // hdop
  // vdop
  // vn
  // ve
  // vd
  // speed_accuracy
  // horiz_accuracy
  // vert_accuracy
  // satellites_visible
  // yaw
  return true;
}

void
mavros_msgs__msg__GPSINPUT__fini(mavros_msgs__msg__GPSINPUT * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fix_type
  // gps_id
  // ignore_flags
  // time_week_ms
  // time_week
  // lat
  // lon
  // alt
  // hdop
  // vdop
  // vn
  // ve
  // vd
  // speed_accuracy
  // horiz_accuracy
  // vert_accuracy
  // satellites_visible
  // yaw
}

bool
mavros_msgs__msg__GPSINPUT__are_equal(const mavros_msgs__msg__GPSINPUT * lhs, const mavros_msgs__msg__GPSINPUT * rhs)
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
  // fix_type
  if (lhs->fix_type != rhs->fix_type) {
    return false;
  }
  // gps_id
  if (lhs->gps_id != rhs->gps_id) {
    return false;
  }
  // ignore_flags
  if (lhs->ignore_flags != rhs->ignore_flags) {
    return false;
  }
  // time_week_ms
  if (lhs->time_week_ms != rhs->time_week_ms) {
    return false;
  }
  // time_week
  if (lhs->time_week != rhs->time_week) {
    return false;
  }
  // lat
  if (lhs->lat != rhs->lat) {
    return false;
  }
  // lon
  if (lhs->lon != rhs->lon) {
    return false;
  }
  // alt
  if (lhs->alt != rhs->alt) {
    return false;
  }
  // hdop
  if (lhs->hdop != rhs->hdop) {
    return false;
  }
  // vdop
  if (lhs->vdop != rhs->vdop) {
    return false;
  }
  // vn
  if (lhs->vn != rhs->vn) {
    return false;
  }
  // ve
  if (lhs->ve != rhs->ve) {
    return false;
  }
  // vd
  if (lhs->vd != rhs->vd) {
    return false;
  }
  // speed_accuracy
  if (lhs->speed_accuracy != rhs->speed_accuracy) {
    return false;
  }
  // horiz_accuracy
  if (lhs->horiz_accuracy != rhs->horiz_accuracy) {
    return false;
  }
  // vert_accuracy
  if (lhs->vert_accuracy != rhs->vert_accuracy) {
    return false;
  }
  // satellites_visible
  if (lhs->satellites_visible != rhs->satellites_visible) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__GPSINPUT__copy(
  const mavros_msgs__msg__GPSINPUT * input,
  mavros_msgs__msg__GPSINPUT * output)
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
  // fix_type
  output->fix_type = input->fix_type;
  // gps_id
  output->gps_id = input->gps_id;
  // ignore_flags
  output->ignore_flags = input->ignore_flags;
  // time_week_ms
  output->time_week_ms = input->time_week_ms;
  // time_week
  output->time_week = input->time_week;
  // lat
  output->lat = input->lat;
  // lon
  output->lon = input->lon;
  // alt
  output->alt = input->alt;
  // hdop
  output->hdop = input->hdop;
  // vdop
  output->vdop = input->vdop;
  // vn
  output->vn = input->vn;
  // ve
  output->ve = input->ve;
  // vd
  output->vd = input->vd;
  // speed_accuracy
  output->speed_accuracy = input->speed_accuracy;
  // horiz_accuracy
  output->horiz_accuracy = input->horiz_accuracy;
  // vert_accuracy
  output->vert_accuracy = input->vert_accuracy;
  // satellites_visible
  output->satellites_visible = input->satellites_visible;
  // yaw
  output->yaw = input->yaw;
  return true;
}

mavros_msgs__msg__GPSINPUT *
mavros_msgs__msg__GPSINPUT__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSINPUT * msg = (mavros_msgs__msg__GPSINPUT *)allocator.allocate(sizeof(mavros_msgs__msg__GPSINPUT), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__GPSINPUT));
  bool success = mavros_msgs__msg__GPSINPUT__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__GPSINPUT__destroy(mavros_msgs__msg__GPSINPUT * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__GPSINPUT__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__GPSINPUT__Sequence__init(mavros_msgs__msg__GPSINPUT__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSINPUT * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__GPSINPUT *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__GPSINPUT), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__GPSINPUT__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__GPSINPUT__fini(&data[i - 1]);
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
mavros_msgs__msg__GPSINPUT__Sequence__fini(mavros_msgs__msg__GPSINPUT__Sequence * array)
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
      mavros_msgs__msg__GPSINPUT__fini(&array->data[i]);
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

mavros_msgs__msg__GPSINPUT__Sequence *
mavros_msgs__msg__GPSINPUT__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSINPUT__Sequence * array = (mavros_msgs__msg__GPSINPUT__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__GPSINPUT__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__GPSINPUT__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__GPSINPUT__Sequence__destroy(mavros_msgs__msg__GPSINPUT__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__GPSINPUT__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__GPSINPUT__Sequence__are_equal(const mavros_msgs__msg__GPSINPUT__Sequence * lhs, const mavros_msgs__msg__GPSINPUT__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__GPSINPUT__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__GPSINPUT__Sequence__copy(
  const mavros_msgs__msg__GPSINPUT__Sequence * input,
  mavros_msgs__msg__GPSINPUT__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__GPSINPUT);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__GPSINPUT * data =
      (mavros_msgs__msg__GPSINPUT *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__GPSINPUT__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__GPSINPUT__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__GPSINPUT__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
