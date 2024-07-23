// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/Param.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/param__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `param_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `value`
#include "mavros_msgs/msg/detail/param_value__functions.h"

bool
mavros_msgs__msg__Param__init(mavros_msgs__msg__Param * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__Param__fini(msg);
    return false;
  }
  // param_id
  if (!rosidl_runtime_c__String__init(&msg->param_id)) {
    mavros_msgs__msg__Param__fini(msg);
    return false;
  }
  // value
  if (!mavros_msgs__msg__ParamValue__init(&msg->value)) {
    mavros_msgs__msg__Param__fini(msg);
    return false;
  }
  // param_index
  // param_count
  return true;
}

void
mavros_msgs__msg__Param__fini(mavros_msgs__msg__Param * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // param_id
  rosidl_runtime_c__String__fini(&msg->param_id);
  // value
  mavros_msgs__msg__ParamValue__fini(&msg->value);
  // param_index
  // param_count
}

bool
mavros_msgs__msg__Param__are_equal(const mavros_msgs__msg__Param * lhs, const mavros_msgs__msg__Param * rhs)
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
  // param_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->param_id), &(rhs->param_id)))
  {
    return false;
  }
  // value
  if (!mavros_msgs__msg__ParamValue__are_equal(
      &(lhs->value), &(rhs->value)))
  {
    return false;
  }
  // param_index
  if (lhs->param_index != rhs->param_index) {
    return false;
  }
  // param_count
  if (lhs->param_count != rhs->param_count) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__Param__copy(
  const mavros_msgs__msg__Param * input,
  mavros_msgs__msg__Param * output)
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
  // param_id
  if (!rosidl_runtime_c__String__copy(
      &(input->param_id), &(output->param_id)))
  {
    return false;
  }
  // value
  if (!mavros_msgs__msg__ParamValue__copy(
      &(input->value), &(output->value)))
  {
    return false;
  }
  // param_index
  output->param_index = input->param_index;
  // param_count
  output->param_count = input->param_count;
  return true;
}

mavros_msgs__msg__Param *
mavros_msgs__msg__Param__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Param * msg = (mavros_msgs__msg__Param *)allocator.allocate(sizeof(mavros_msgs__msg__Param), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__Param));
  bool success = mavros_msgs__msg__Param__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__Param__destroy(mavros_msgs__msg__Param * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__Param__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__Param__Sequence__init(mavros_msgs__msg__Param__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Param * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__Param *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__Param), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__Param__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__Param__fini(&data[i - 1]);
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
mavros_msgs__msg__Param__Sequence__fini(mavros_msgs__msg__Param__Sequence * array)
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
      mavros_msgs__msg__Param__fini(&array->data[i]);
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

mavros_msgs__msg__Param__Sequence *
mavros_msgs__msg__Param__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Param__Sequence * array = (mavros_msgs__msg__Param__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__Param__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__Param__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__Param__Sequence__destroy(mavros_msgs__msg__Param__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__Param__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__Param__Sequence__are_equal(const mavros_msgs__msg__Param__Sequence * lhs, const mavros_msgs__msg__Param__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__Param__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Param__Sequence__copy(
  const mavros_msgs__msg__Param__Sequence * input,
  mavros_msgs__msg__Param__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__Param);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__Param * data =
      (mavros_msgs__msg__Param *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__Param__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__Param__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__Param__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
