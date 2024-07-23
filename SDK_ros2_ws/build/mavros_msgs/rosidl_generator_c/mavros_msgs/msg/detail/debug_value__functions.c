// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/DebugValue.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/debug_value__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mavros_msgs__msg__DebugValue__init(mavros_msgs__msg__DebugValue * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__DebugValue__fini(msg);
    return false;
  }
  // index
  // array_id
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    mavros_msgs__msg__DebugValue__fini(msg);
    return false;
  }
  // value_float
  // value_int
  // data
  if (!rosidl_runtime_c__float__Sequence__init(&msg->data, 0)) {
    mavros_msgs__msg__DebugValue__fini(msg);
    return false;
  }
  // type
  return true;
}

void
mavros_msgs__msg__DebugValue__fini(mavros_msgs__msg__DebugValue * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // index
  // array_id
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // value_float
  // value_int
  // data
  rosidl_runtime_c__float__Sequence__fini(&msg->data);
  // type
}

bool
mavros_msgs__msg__DebugValue__are_equal(const mavros_msgs__msg__DebugValue * lhs, const mavros_msgs__msg__DebugValue * rhs)
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
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // array_id
  if (lhs->array_id != rhs->array_id) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // value_float
  if (lhs->value_float != rhs->value_float) {
    return false;
  }
  // value_int
  if (lhs->value_int != rhs->value_int) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__DebugValue__copy(
  const mavros_msgs__msg__DebugValue * input,
  mavros_msgs__msg__DebugValue * output)
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
  // index
  output->index = input->index;
  // array_id
  output->array_id = input->array_id;
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // value_float
  output->value_float = input->value_float;
  // value_int
  output->value_int = input->value_int;
  // data
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // type
  output->type = input->type;
  return true;
}

mavros_msgs__msg__DebugValue *
mavros_msgs__msg__DebugValue__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__DebugValue * msg = (mavros_msgs__msg__DebugValue *)allocator.allocate(sizeof(mavros_msgs__msg__DebugValue), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__DebugValue));
  bool success = mavros_msgs__msg__DebugValue__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__DebugValue__destroy(mavros_msgs__msg__DebugValue * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__DebugValue__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__DebugValue__Sequence__init(mavros_msgs__msg__DebugValue__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__DebugValue * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__DebugValue *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__DebugValue), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__DebugValue__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__DebugValue__fini(&data[i - 1]);
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
mavros_msgs__msg__DebugValue__Sequence__fini(mavros_msgs__msg__DebugValue__Sequence * array)
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
      mavros_msgs__msg__DebugValue__fini(&array->data[i]);
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

mavros_msgs__msg__DebugValue__Sequence *
mavros_msgs__msg__DebugValue__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__DebugValue__Sequence * array = (mavros_msgs__msg__DebugValue__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__DebugValue__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__DebugValue__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__DebugValue__Sequence__destroy(mavros_msgs__msg__DebugValue__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__DebugValue__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__DebugValue__Sequence__are_equal(const mavros_msgs__msg__DebugValue__Sequence * lhs, const mavros_msgs__msg__DebugValue__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__DebugValue__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__DebugValue__Sequence__copy(
  const mavros_msgs__msg__DebugValue__Sequence * input,
  mavros_msgs__msg__DebugValue__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__DebugValue);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__DebugValue * data =
      (mavros_msgs__msg__DebugValue *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__DebugValue__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__DebugValue__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__DebugValue__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
