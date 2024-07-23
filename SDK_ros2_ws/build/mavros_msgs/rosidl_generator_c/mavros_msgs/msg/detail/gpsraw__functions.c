// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/GPSRAW.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/gpsraw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__GPSRAW__init(mavros_msgs__msg__GPSRAW * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__GPSRAW__fini(msg);
    return false;
  }
  // fix_type
  // lat
  // lon
  // alt
  // eph
  // epv
  // vel
  // cog
  // satellites_visible
  // alt_ellipsoid
  // h_acc
  // v_acc
  // vel_acc
  // hdg_acc
  // yaw
  // dgps_numch
  // dgps_age
  return true;
}

void
mavros_msgs__msg__GPSRAW__fini(mavros_msgs__msg__GPSRAW * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fix_type
  // lat
  // lon
  // alt
  // eph
  // epv
  // vel
  // cog
  // satellites_visible
  // alt_ellipsoid
  // h_acc
  // v_acc
  // vel_acc
  // hdg_acc
  // yaw
  // dgps_numch
  // dgps_age
}

bool
mavros_msgs__msg__GPSRAW__are_equal(const mavros_msgs__msg__GPSRAW * lhs, const mavros_msgs__msg__GPSRAW * rhs)
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
  // eph
  if (lhs->eph != rhs->eph) {
    return false;
  }
  // epv
  if (lhs->epv != rhs->epv) {
    return false;
  }
  // vel
  if (lhs->vel != rhs->vel) {
    return false;
  }
  // cog
  if (lhs->cog != rhs->cog) {
    return false;
  }
  // satellites_visible
  if (lhs->satellites_visible != rhs->satellites_visible) {
    return false;
  }
  // alt_ellipsoid
  if (lhs->alt_ellipsoid != rhs->alt_ellipsoid) {
    return false;
  }
  // h_acc
  if (lhs->h_acc != rhs->h_acc) {
    return false;
  }
  // v_acc
  if (lhs->v_acc != rhs->v_acc) {
    return false;
  }
  // vel_acc
  if (lhs->vel_acc != rhs->vel_acc) {
    return false;
  }
  // hdg_acc
  if (lhs->hdg_acc != rhs->hdg_acc) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // dgps_numch
  if (lhs->dgps_numch != rhs->dgps_numch) {
    return false;
  }
  // dgps_age
  if (lhs->dgps_age != rhs->dgps_age) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__GPSRAW__copy(
  const mavros_msgs__msg__GPSRAW * input,
  mavros_msgs__msg__GPSRAW * output)
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
  // lat
  output->lat = input->lat;
  // lon
  output->lon = input->lon;
  // alt
  output->alt = input->alt;
  // eph
  output->eph = input->eph;
  // epv
  output->epv = input->epv;
  // vel
  output->vel = input->vel;
  // cog
  output->cog = input->cog;
  // satellites_visible
  output->satellites_visible = input->satellites_visible;
  // alt_ellipsoid
  output->alt_ellipsoid = input->alt_ellipsoid;
  // h_acc
  output->h_acc = input->h_acc;
  // v_acc
  output->v_acc = input->v_acc;
  // vel_acc
  output->vel_acc = input->vel_acc;
  // hdg_acc
  output->hdg_acc = input->hdg_acc;
  // yaw
  output->yaw = input->yaw;
  // dgps_numch
  output->dgps_numch = input->dgps_numch;
  // dgps_age
  output->dgps_age = input->dgps_age;
  return true;
}

mavros_msgs__msg__GPSRAW *
mavros_msgs__msg__GPSRAW__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSRAW * msg = (mavros_msgs__msg__GPSRAW *)allocator.allocate(sizeof(mavros_msgs__msg__GPSRAW), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__GPSRAW));
  bool success = mavros_msgs__msg__GPSRAW__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__GPSRAW__destroy(mavros_msgs__msg__GPSRAW * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__GPSRAW__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__GPSRAW__Sequence__init(mavros_msgs__msg__GPSRAW__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSRAW * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__GPSRAW *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__GPSRAW), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__GPSRAW__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__GPSRAW__fini(&data[i - 1]);
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
mavros_msgs__msg__GPSRAW__Sequence__fini(mavros_msgs__msg__GPSRAW__Sequence * array)
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
      mavros_msgs__msg__GPSRAW__fini(&array->data[i]);
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

mavros_msgs__msg__GPSRAW__Sequence *
mavros_msgs__msg__GPSRAW__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSRAW__Sequence * array = (mavros_msgs__msg__GPSRAW__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__GPSRAW__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__GPSRAW__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__GPSRAW__Sequence__destroy(mavros_msgs__msg__GPSRAW__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__GPSRAW__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__GPSRAW__Sequence__are_equal(const mavros_msgs__msg__GPSRAW__Sequence * lhs, const mavros_msgs__msg__GPSRAW__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__GPSRAW__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__GPSRAW__Sequence__copy(
  const mavros_msgs__msg__GPSRAW__Sequence * input,
  mavros_msgs__msg__GPSRAW__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__GPSRAW);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__GPSRAW * data =
      (mavros_msgs__msg__GPSRAW *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__GPSRAW__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__GPSRAW__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__GPSRAW__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
