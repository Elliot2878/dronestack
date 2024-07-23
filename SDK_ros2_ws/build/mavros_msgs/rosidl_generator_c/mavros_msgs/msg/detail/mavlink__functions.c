// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/Mavlink.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/mavlink__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `payload64`
// Member `signature`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mavros_msgs__msg__Mavlink__init(mavros_msgs__msg__Mavlink * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__Mavlink__fini(msg);
    return false;
  }
  // framing_status
  // magic
  // len
  // incompat_flags
  // compat_flags
  // seq
  // sysid
  // compid
  // msgid
  // checksum
  // payload64
  if (!rosidl_runtime_c__uint64__Sequence__init(&msg->payload64, 0)) {
    mavros_msgs__msg__Mavlink__fini(msg);
    return false;
  }
  // signature
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->signature, 0)) {
    mavros_msgs__msg__Mavlink__fini(msg);
    return false;
  }
  return true;
}

void
mavros_msgs__msg__Mavlink__fini(mavros_msgs__msg__Mavlink * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // framing_status
  // magic
  // len
  // incompat_flags
  // compat_flags
  // seq
  // sysid
  // compid
  // msgid
  // checksum
  // payload64
  rosidl_runtime_c__uint64__Sequence__fini(&msg->payload64);
  // signature
  rosidl_runtime_c__uint8__Sequence__fini(&msg->signature);
}

bool
mavros_msgs__msg__Mavlink__are_equal(const mavros_msgs__msg__Mavlink * lhs, const mavros_msgs__msg__Mavlink * rhs)
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
  // framing_status
  if (lhs->framing_status != rhs->framing_status) {
    return false;
  }
  // magic
  if (lhs->magic != rhs->magic) {
    return false;
  }
  // len
  if (lhs->len != rhs->len) {
    return false;
  }
  // incompat_flags
  if (lhs->incompat_flags != rhs->incompat_flags) {
    return false;
  }
  // compat_flags
  if (lhs->compat_flags != rhs->compat_flags) {
    return false;
  }
  // seq
  if (lhs->seq != rhs->seq) {
    return false;
  }
  // sysid
  if (lhs->sysid != rhs->sysid) {
    return false;
  }
  // compid
  if (lhs->compid != rhs->compid) {
    return false;
  }
  // msgid
  if (lhs->msgid != rhs->msgid) {
    return false;
  }
  // checksum
  if (lhs->checksum != rhs->checksum) {
    return false;
  }
  // payload64
  if (!rosidl_runtime_c__uint64__Sequence__are_equal(
      &(lhs->payload64), &(rhs->payload64)))
  {
    return false;
  }
  // signature
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->signature), &(rhs->signature)))
  {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__Mavlink__copy(
  const mavros_msgs__msg__Mavlink * input,
  mavros_msgs__msg__Mavlink * output)
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
  // framing_status
  output->framing_status = input->framing_status;
  // magic
  output->magic = input->magic;
  // len
  output->len = input->len;
  // incompat_flags
  output->incompat_flags = input->incompat_flags;
  // compat_flags
  output->compat_flags = input->compat_flags;
  // seq
  output->seq = input->seq;
  // sysid
  output->sysid = input->sysid;
  // compid
  output->compid = input->compid;
  // msgid
  output->msgid = input->msgid;
  // checksum
  output->checksum = input->checksum;
  // payload64
  if (!rosidl_runtime_c__uint64__Sequence__copy(
      &(input->payload64), &(output->payload64)))
  {
    return false;
  }
  // signature
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->signature), &(output->signature)))
  {
    return false;
  }
  return true;
}

mavros_msgs__msg__Mavlink *
mavros_msgs__msg__Mavlink__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Mavlink * msg = (mavros_msgs__msg__Mavlink *)allocator.allocate(sizeof(mavros_msgs__msg__Mavlink), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__Mavlink));
  bool success = mavros_msgs__msg__Mavlink__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__Mavlink__destroy(mavros_msgs__msg__Mavlink * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__Mavlink__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__Mavlink__Sequence__init(mavros_msgs__msg__Mavlink__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Mavlink * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__Mavlink *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__Mavlink), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__Mavlink__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__Mavlink__fini(&data[i - 1]);
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
mavros_msgs__msg__Mavlink__Sequence__fini(mavros_msgs__msg__Mavlink__Sequence * array)
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
      mavros_msgs__msg__Mavlink__fini(&array->data[i]);
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

mavros_msgs__msg__Mavlink__Sequence *
mavros_msgs__msg__Mavlink__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__Mavlink__Sequence * array = (mavros_msgs__msg__Mavlink__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__Mavlink__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__Mavlink__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__Mavlink__Sequence__destroy(mavros_msgs__msg__Mavlink__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__Mavlink__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__Mavlink__Sequence__are_equal(const mavros_msgs__msg__Mavlink__Sequence * lhs, const mavros_msgs__msg__Mavlink__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__Mavlink__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__Mavlink__Sequence__copy(
  const mavros_msgs__msg__Mavlink__Sequence * input,
  mavros_msgs__msg__Mavlink__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__Mavlink);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__Mavlink * data =
      (mavros_msgs__msg__Mavlink *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__Mavlink__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__Mavlink__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__Mavlink__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
