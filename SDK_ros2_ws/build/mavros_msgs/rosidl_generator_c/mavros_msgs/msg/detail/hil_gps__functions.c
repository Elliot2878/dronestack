// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/HilGPS.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/hil_gps__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `geo`
#include "geographic_msgs/msg/detail/geo_point__functions.h"

bool
mavros_msgs__msg__HilGPS__init(mavros_msgs__msg__HilGPS * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__HilGPS__fini(msg);
    return false;
  }
  // fix_type
  // geo
  if (!geographic_msgs__msg__GeoPoint__init(&msg->geo)) {
    mavros_msgs__msg__HilGPS__fini(msg);
    return false;
  }
  // eph
  // epv
  // vel
  // vn
  // ve
  // vd
  // cog
  // satellites_visible
  return true;
}

void
mavros_msgs__msg__HilGPS__fini(mavros_msgs__msg__HilGPS * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fix_type
  // geo
  geographic_msgs__msg__GeoPoint__fini(&msg->geo);
  // eph
  // epv
  // vel
  // vn
  // ve
  // vd
  // cog
  // satellites_visible
}

bool
mavros_msgs__msg__HilGPS__are_equal(const mavros_msgs__msg__HilGPS * lhs, const mavros_msgs__msg__HilGPS * rhs)
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
  // geo
  if (!geographic_msgs__msg__GeoPoint__are_equal(
      &(lhs->geo), &(rhs->geo)))
  {
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
  // cog
  if (lhs->cog != rhs->cog) {
    return false;
  }
  // satellites_visible
  if (lhs->satellites_visible != rhs->satellites_visible) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__HilGPS__copy(
  const mavros_msgs__msg__HilGPS * input,
  mavros_msgs__msg__HilGPS * output)
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
  // geo
  if (!geographic_msgs__msg__GeoPoint__copy(
      &(input->geo), &(output->geo)))
  {
    return false;
  }
  // eph
  output->eph = input->eph;
  // epv
  output->epv = input->epv;
  // vel
  output->vel = input->vel;
  // vn
  output->vn = input->vn;
  // ve
  output->ve = input->ve;
  // vd
  output->vd = input->vd;
  // cog
  output->cog = input->cog;
  // satellites_visible
  output->satellites_visible = input->satellites_visible;
  return true;
}

mavros_msgs__msg__HilGPS *
mavros_msgs__msg__HilGPS__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilGPS * msg = (mavros_msgs__msg__HilGPS *)allocator.allocate(sizeof(mavros_msgs__msg__HilGPS), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__HilGPS));
  bool success = mavros_msgs__msg__HilGPS__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__HilGPS__destroy(mavros_msgs__msg__HilGPS * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__HilGPS__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__HilGPS__Sequence__init(mavros_msgs__msg__HilGPS__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilGPS * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__HilGPS *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__HilGPS), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__HilGPS__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__HilGPS__fini(&data[i - 1]);
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
mavros_msgs__msg__HilGPS__Sequence__fini(mavros_msgs__msg__HilGPS__Sequence * array)
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
      mavros_msgs__msg__HilGPS__fini(&array->data[i]);
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

mavros_msgs__msg__HilGPS__Sequence *
mavros_msgs__msg__HilGPS__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilGPS__Sequence * array = (mavros_msgs__msg__HilGPS__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__HilGPS__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__HilGPS__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__HilGPS__Sequence__destroy(mavros_msgs__msg__HilGPS__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__HilGPS__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__HilGPS__Sequence__are_equal(const mavros_msgs__msg__HilGPS__Sequence * lhs, const mavros_msgs__msg__HilGPS__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__HilGPS__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__HilGPS__Sequence__copy(
  const mavros_msgs__msg__HilGPS__Sequence * input,
  mavros_msgs__msg__HilGPS__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__HilGPS);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__HilGPS * data =
      (mavros_msgs__msg__HilGPS *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__HilGPS__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__HilGPS__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__HilGPS__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
