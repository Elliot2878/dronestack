// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/HilStateQuaternion.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/hil_state_quaternion__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `angular_velocity`
// Member `linear_acceleration`
// Member `linear_velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `geo`
#include "geographic_msgs/msg/detail/geo_point__functions.h"

bool
mavros_msgs__msg__HilStateQuaternion__init(mavros_msgs__msg__HilStateQuaternion * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
    return false;
  }
  // linear_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_velocity)) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
    return false;
  }
  // geo
  if (!geographic_msgs__msg__GeoPoint__init(&msg->geo)) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
    return false;
  }
  // ind_airspeed
  // true_airspeed
  return true;
}

void
mavros_msgs__msg__HilStateQuaternion__fini(mavros_msgs__msg__HilStateQuaternion * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // linear_velocity
  geometry_msgs__msg__Vector3__fini(&msg->linear_velocity);
  // geo
  geographic_msgs__msg__GeoPoint__fini(&msg->geo);
  // ind_airspeed
  // true_airspeed
}

bool
mavros_msgs__msg__HilStateQuaternion__are_equal(const mavros_msgs__msg__HilStateQuaternion * lhs, const mavros_msgs__msg__HilStateQuaternion * rhs)
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
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->linear_acceleration), &(rhs->linear_acceleration)))
  {
    return false;
  }
  // linear_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->linear_velocity), &(rhs->linear_velocity)))
  {
    return false;
  }
  // geo
  if (!geographic_msgs__msg__GeoPoint__are_equal(
      &(lhs->geo), &(rhs->geo)))
  {
    return false;
  }
  // ind_airspeed
  if (lhs->ind_airspeed != rhs->ind_airspeed) {
    return false;
  }
  // true_airspeed
  if (lhs->true_airspeed != rhs->true_airspeed) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__HilStateQuaternion__copy(
  const mavros_msgs__msg__HilStateQuaternion * input,
  mavros_msgs__msg__HilStateQuaternion * output)
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
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->linear_acceleration), &(output->linear_acceleration)))
  {
    return false;
  }
  // linear_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->linear_velocity), &(output->linear_velocity)))
  {
    return false;
  }
  // geo
  if (!geographic_msgs__msg__GeoPoint__copy(
      &(input->geo), &(output->geo)))
  {
    return false;
  }
  // ind_airspeed
  output->ind_airspeed = input->ind_airspeed;
  // true_airspeed
  output->true_airspeed = input->true_airspeed;
  return true;
}

mavros_msgs__msg__HilStateQuaternion *
mavros_msgs__msg__HilStateQuaternion__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilStateQuaternion * msg = (mavros_msgs__msg__HilStateQuaternion *)allocator.allocate(sizeof(mavros_msgs__msg__HilStateQuaternion), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__HilStateQuaternion));
  bool success = mavros_msgs__msg__HilStateQuaternion__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__HilStateQuaternion__destroy(mavros_msgs__msg__HilStateQuaternion * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__HilStateQuaternion__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__HilStateQuaternion__Sequence__init(mavros_msgs__msg__HilStateQuaternion__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilStateQuaternion * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__HilStateQuaternion *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__HilStateQuaternion), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__HilStateQuaternion__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__HilStateQuaternion__fini(&data[i - 1]);
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
mavros_msgs__msg__HilStateQuaternion__Sequence__fini(mavros_msgs__msg__HilStateQuaternion__Sequence * array)
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
      mavros_msgs__msg__HilStateQuaternion__fini(&array->data[i]);
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

mavros_msgs__msg__HilStateQuaternion__Sequence *
mavros_msgs__msg__HilStateQuaternion__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilStateQuaternion__Sequence * array = (mavros_msgs__msg__HilStateQuaternion__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__HilStateQuaternion__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__HilStateQuaternion__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__HilStateQuaternion__Sequence__destroy(mavros_msgs__msg__HilStateQuaternion__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__HilStateQuaternion__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__HilStateQuaternion__Sequence__are_equal(const mavros_msgs__msg__HilStateQuaternion__Sequence * lhs, const mavros_msgs__msg__HilStateQuaternion__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__HilStateQuaternion__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__HilStateQuaternion__Sequence__copy(
  const mavros_msgs__msg__HilStateQuaternion__Sequence * input,
  mavros_msgs__msg__HilStateQuaternion__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__HilStateQuaternion);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__HilStateQuaternion * data =
      (mavros_msgs__msg__HilStateQuaternion *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__HilStateQuaternion__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__HilStateQuaternion__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__HilStateQuaternion__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
