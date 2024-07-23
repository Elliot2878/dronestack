// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/HilSensor.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/hil_sensor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `acc`
// Member `gyro`
// Member `mag`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mavros_msgs__msg__HilSensor__init(mavros_msgs__msg__HilSensor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__HilSensor__fini(msg);
    return false;
  }
  // acc
  if (!geometry_msgs__msg__Vector3__init(&msg->acc)) {
    mavros_msgs__msg__HilSensor__fini(msg);
    return false;
  }
  // gyro
  if (!geometry_msgs__msg__Vector3__init(&msg->gyro)) {
    mavros_msgs__msg__HilSensor__fini(msg);
    return false;
  }
  // mag
  if (!geometry_msgs__msg__Vector3__init(&msg->mag)) {
    mavros_msgs__msg__HilSensor__fini(msg);
    return false;
  }
  // abs_pressure
  // diff_pressure
  // pressure_alt
  // temperature
  // fields_updated
  return true;
}

void
mavros_msgs__msg__HilSensor__fini(mavros_msgs__msg__HilSensor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // acc
  geometry_msgs__msg__Vector3__fini(&msg->acc);
  // gyro
  geometry_msgs__msg__Vector3__fini(&msg->gyro);
  // mag
  geometry_msgs__msg__Vector3__fini(&msg->mag);
  // abs_pressure
  // diff_pressure
  // pressure_alt
  // temperature
  // fields_updated
}

bool
mavros_msgs__msg__HilSensor__are_equal(const mavros_msgs__msg__HilSensor * lhs, const mavros_msgs__msg__HilSensor * rhs)
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
  // acc
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acc), &(rhs->acc)))
  {
    return false;
  }
  // gyro
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->gyro), &(rhs->gyro)))
  {
    return false;
  }
  // mag
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->mag), &(rhs->mag)))
  {
    return false;
  }
  // abs_pressure
  if (lhs->abs_pressure != rhs->abs_pressure) {
    return false;
  }
  // diff_pressure
  if (lhs->diff_pressure != rhs->diff_pressure) {
    return false;
  }
  // pressure_alt
  if (lhs->pressure_alt != rhs->pressure_alt) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // fields_updated
  if (lhs->fields_updated != rhs->fields_updated) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__HilSensor__copy(
  const mavros_msgs__msg__HilSensor * input,
  mavros_msgs__msg__HilSensor * output)
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
  // acc
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acc), &(output->acc)))
  {
    return false;
  }
  // gyro
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->gyro), &(output->gyro)))
  {
    return false;
  }
  // mag
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->mag), &(output->mag)))
  {
    return false;
  }
  // abs_pressure
  output->abs_pressure = input->abs_pressure;
  // diff_pressure
  output->diff_pressure = input->diff_pressure;
  // pressure_alt
  output->pressure_alt = input->pressure_alt;
  // temperature
  output->temperature = input->temperature;
  // fields_updated
  output->fields_updated = input->fields_updated;
  return true;
}

mavros_msgs__msg__HilSensor *
mavros_msgs__msg__HilSensor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilSensor * msg = (mavros_msgs__msg__HilSensor *)allocator.allocate(sizeof(mavros_msgs__msg__HilSensor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__HilSensor));
  bool success = mavros_msgs__msg__HilSensor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__HilSensor__destroy(mavros_msgs__msg__HilSensor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__HilSensor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__HilSensor__Sequence__init(mavros_msgs__msg__HilSensor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilSensor * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__HilSensor *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__HilSensor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__HilSensor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__HilSensor__fini(&data[i - 1]);
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
mavros_msgs__msg__HilSensor__Sequence__fini(mavros_msgs__msg__HilSensor__Sequence * array)
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
      mavros_msgs__msg__HilSensor__fini(&array->data[i]);
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

mavros_msgs__msg__HilSensor__Sequence *
mavros_msgs__msg__HilSensor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__HilSensor__Sequence * array = (mavros_msgs__msg__HilSensor__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__HilSensor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__HilSensor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__HilSensor__Sequence__destroy(mavros_msgs__msg__HilSensor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__HilSensor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__HilSensor__Sequence__are_equal(const mavros_msgs__msg__HilSensor__Sequence * lhs, const mavros_msgs__msg__HilSensor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__HilSensor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__HilSensor__Sequence__copy(
  const mavros_msgs__msg__HilSensor__Sequence * input,
  mavros_msgs__msg__HilSensor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__HilSensor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__HilSensor * data =
      (mavros_msgs__msg__HilSensor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__HilSensor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__HilSensor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__HilSensor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
