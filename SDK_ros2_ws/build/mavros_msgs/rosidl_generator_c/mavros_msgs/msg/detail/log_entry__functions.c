// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/LogEntry.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/log_entry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `time_utc`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
mavros_msgs__msg__LogEntry__init(mavros_msgs__msg__LogEntry * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__LogEntry__fini(msg);
    return false;
  }
  // id
  // num_logs
  // last_log_num
  // time_utc
  if (!builtin_interfaces__msg__Time__init(&msg->time_utc)) {
    mavros_msgs__msg__LogEntry__fini(msg);
    return false;
  }
  // size
  return true;
}

void
mavros_msgs__msg__LogEntry__fini(mavros_msgs__msg__LogEntry * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // num_logs
  // last_log_num
  // time_utc
  builtin_interfaces__msg__Time__fini(&msg->time_utc);
  // size
}

bool
mavros_msgs__msg__LogEntry__are_equal(const mavros_msgs__msg__LogEntry * lhs, const mavros_msgs__msg__LogEntry * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // num_logs
  if (lhs->num_logs != rhs->num_logs) {
    return false;
  }
  // last_log_num
  if (lhs->last_log_num != rhs->last_log_num) {
    return false;
  }
  // time_utc
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->time_utc), &(rhs->time_utc)))
  {
    return false;
  }
  // size
  if (lhs->size != rhs->size) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__LogEntry__copy(
  const mavros_msgs__msg__LogEntry * input,
  mavros_msgs__msg__LogEntry * output)
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
  // id
  output->id = input->id;
  // num_logs
  output->num_logs = input->num_logs;
  // last_log_num
  output->last_log_num = input->last_log_num;
  // time_utc
  if (!builtin_interfaces__msg__Time__copy(
      &(input->time_utc), &(output->time_utc)))
  {
    return false;
  }
  // size
  output->size = input->size;
  return true;
}

mavros_msgs__msg__LogEntry *
mavros_msgs__msg__LogEntry__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__LogEntry * msg = (mavros_msgs__msg__LogEntry *)allocator.allocate(sizeof(mavros_msgs__msg__LogEntry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__LogEntry));
  bool success = mavros_msgs__msg__LogEntry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__LogEntry__destroy(mavros_msgs__msg__LogEntry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__LogEntry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__LogEntry__Sequence__init(mavros_msgs__msg__LogEntry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__LogEntry * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__LogEntry *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__LogEntry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__LogEntry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__LogEntry__fini(&data[i - 1]);
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
mavros_msgs__msg__LogEntry__Sequence__fini(mavros_msgs__msg__LogEntry__Sequence * array)
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
      mavros_msgs__msg__LogEntry__fini(&array->data[i]);
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

mavros_msgs__msg__LogEntry__Sequence *
mavros_msgs__msg__LogEntry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__LogEntry__Sequence * array = (mavros_msgs__msg__LogEntry__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__LogEntry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__LogEntry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__LogEntry__Sequence__destroy(mavros_msgs__msg__LogEntry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__LogEntry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__LogEntry__Sequence__are_equal(const mavros_msgs__msg__LogEntry__Sequence * lhs, const mavros_msgs__msg__LogEntry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__LogEntry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__LogEntry__Sequence__copy(
  const mavros_msgs__msg__LogEntry__Sequence * input,
  mavros_msgs__msg__LogEntry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__LogEntry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__LogEntry * data =
      (mavros_msgs__msg__LogEntry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__LogEntry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__LogEntry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__LogEntry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
