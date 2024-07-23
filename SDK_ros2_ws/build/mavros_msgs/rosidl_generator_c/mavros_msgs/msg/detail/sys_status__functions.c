// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/SysStatus.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/sys_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__SysStatus__init(mavros_msgs__msg__SysStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__SysStatus__fini(msg);
    return false;
  }
  // sensors_present
  // sensors_enabled
  // sensors_health
  // load
  // voltage_battery
  // current_battery
  // battery_remaining
  // drop_rate_comm
  // errors_comm
  // errors_count1
  // errors_count2
  // errors_count3
  // errors_count4
  return true;
}

void
mavros_msgs__msg__SysStatus__fini(mavros_msgs__msg__SysStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sensors_present
  // sensors_enabled
  // sensors_health
  // load
  // voltage_battery
  // current_battery
  // battery_remaining
  // drop_rate_comm
  // errors_comm
  // errors_count1
  // errors_count2
  // errors_count3
  // errors_count4
}

bool
mavros_msgs__msg__SysStatus__are_equal(const mavros_msgs__msg__SysStatus * lhs, const mavros_msgs__msg__SysStatus * rhs)
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
  // sensors_present
  if (lhs->sensors_present != rhs->sensors_present) {
    return false;
  }
  // sensors_enabled
  if (lhs->sensors_enabled != rhs->sensors_enabled) {
    return false;
  }
  // sensors_health
  if (lhs->sensors_health != rhs->sensors_health) {
    return false;
  }
  // load
  if (lhs->load != rhs->load) {
    return false;
  }
  // voltage_battery
  if (lhs->voltage_battery != rhs->voltage_battery) {
    return false;
  }
  // current_battery
  if (lhs->current_battery != rhs->current_battery) {
    return false;
  }
  // battery_remaining
  if (lhs->battery_remaining != rhs->battery_remaining) {
    return false;
  }
  // drop_rate_comm
  if (lhs->drop_rate_comm != rhs->drop_rate_comm) {
    return false;
  }
  // errors_comm
  if (lhs->errors_comm != rhs->errors_comm) {
    return false;
  }
  // errors_count1
  if (lhs->errors_count1 != rhs->errors_count1) {
    return false;
  }
  // errors_count2
  if (lhs->errors_count2 != rhs->errors_count2) {
    return false;
  }
  // errors_count3
  if (lhs->errors_count3 != rhs->errors_count3) {
    return false;
  }
  // errors_count4
  if (lhs->errors_count4 != rhs->errors_count4) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__SysStatus__copy(
  const mavros_msgs__msg__SysStatus * input,
  mavros_msgs__msg__SysStatus * output)
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
  // sensors_present
  output->sensors_present = input->sensors_present;
  // sensors_enabled
  output->sensors_enabled = input->sensors_enabled;
  // sensors_health
  output->sensors_health = input->sensors_health;
  // load
  output->load = input->load;
  // voltage_battery
  output->voltage_battery = input->voltage_battery;
  // current_battery
  output->current_battery = input->current_battery;
  // battery_remaining
  output->battery_remaining = input->battery_remaining;
  // drop_rate_comm
  output->drop_rate_comm = input->drop_rate_comm;
  // errors_comm
  output->errors_comm = input->errors_comm;
  // errors_count1
  output->errors_count1 = input->errors_count1;
  // errors_count2
  output->errors_count2 = input->errors_count2;
  // errors_count3
  output->errors_count3 = input->errors_count3;
  // errors_count4
  output->errors_count4 = input->errors_count4;
  return true;
}

mavros_msgs__msg__SysStatus *
mavros_msgs__msg__SysStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__SysStatus * msg = (mavros_msgs__msg__SysStatus *)allocator.allocate(sizeof(mavros_msgs__msg__SysStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__SysStatus));
  bool success = mavros_msgs__msg__SysStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__SysStatus__destroy(mavros_msgs__msg__SysStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__SysStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__SysStatus__Sequence__init(mavros_msgs__msg__SysStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__SysStatus * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__SysStatus *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__SysStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__SysStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__SysStatus__fini(&data[i - 1]);
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
mavros_msgs__msg__SysStatus__Sequence__fini(mavros_msgs__msg__SysStatus__Sequence * array)
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
      mavros_msgs__msg__SysStatus__fini(&array->data[i]);
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

mavros_msgs__msg__SysStatus__Sequence *
mavros_msgs__msg__SysStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__SysStatus__Sequence * array = (mavros_msgs__msg__SysStatus__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__SysStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__SysStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__SysStatus__Sequence__destroy(mavros_msgs__msg__SysStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__SysStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__SysStatus__Sequence__are_equal(const mavros_msgs__msg__SysStatus__Sequence * lhs, const mavros_msgs__msg__SysStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__SysStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__SysStatus__Sequence__copy(
  const mavros_msgs__msg__SysStatus__Sequence * input,
  mavros_msgs__msg__SysStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__SysStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__SysStatus * data =
      (mavros_msgs__msg__SysStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__SysStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__SysStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__SysStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
