// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/GPSRTK.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/gpsrtk__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__GPSRTK__init(mavros_msgs__msg__GPSRTK * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__GPSRTK__fini(msg);
    return false;
  }
  // rtk_receiver_id
  // wn
  // tow
  // rtk_health
  // rtk_rate
  // nsats
  // baseline_a
  // baseline_b
  // baseline_c
  // accuracy
  // iar_num_hypotheses
  return true;
}

void
mavros_msgs__msg__GPSRTK__fini(mavros_msgs__msg__GPSRTK * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rtk_receiver_id
  // wn
  // tow
  // rtk_health
  // rtk_rate
  // nsats
  // baseline_a
  // baseline_b
  // baseline_c
  // accuracy
  // iar_num_hypotheses
}

bool
mavros_msgs__msg__GPSRTK__are_equal(const mavros_msgs__msg__GPSRTK * lhs, const mavros_msgs__msg__GPSRTK * rhs)
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
  // rtk_receiver_id
  if (lhs->rtk_receiver_id != rhs->rtk_receiver_id) {
    return false;
  }
  // wn
  if (lhs->wn != rhs->wn) {
    return false;
  }
  // tow
  if (lhs->tow != rhs->tow) {
    return false;
  }
  // rtk_health
  if (lhs->rtk_health != rhs->rtk_health) {
    return false;
  }
  // rtk_rate
  if (lhs->rtk_rate != rhs->rtk_rate) {
    return false;
  }
  // nsats
  if (lhs->nsats != rhs->nsats) {
    return false;
  }
  // baseline_a
  if (lhs->baseline_a != rhs->baseline_a) {
    return false;
  }
  // baseline_b
  if (lhs->baseline_b != rhs->baseline_b) {
    return false;
  }
  // baseline_c
  if (lhs->baseline_c != rhs->baseline_c) {
    return false;
  }
  // accuracy
  if (lhs->accuracy != rhs->accuracy) {
    return false;
  }
  // iar_num_hypotheses
  if (lhs->iar_num_hypotheses != rhs->iar_num_hypotheses) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__GPSRTK__copy(
  const mavros_msgs__msg__GPSRTK * input,
  mavros_msgs__msg__GPSRTK * output)
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
  // rtk_receiver_id
  output->rtk_receiver_id = input->rtk_receiver_id;
  // wn
  output->wn = input->wn;
  // tow
  output->tow = input->tow;
  // rtk_health
  output->rtk_health = input->rtk_health;
  // rtk_rate
  output->rtk_rate = input->rtk_rate;
  // nsats
  output->nsats = input->nsats;
  // baseline_a
  output->baseline_a = input->baseline_a;
  // baseline_b
  output->baseline_b = input->baseline_b;
  // baseline_c
  output->baseline_c = input->baseline_c;
  // accuracy
  output->accuracy = input->accuracy;
  // iar_num_hypotheses
  output->iar_num_hypotheses = input->iar_num_hypotheses;
  return true;
}

mavros_msgs__msg__GPSRTK *
mavros_msgs__msg__GPSRTK__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSRTK * msg = (mavros_msgs__msg__GPSRTK *)allocator.allocate(sizeof(mavros_msgs__msg__GPSRTK), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__GPSRTK));
  bool success = mavros_msgs__msg__GPSRTK__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__GPSRTK__destroy(mavros_msgs__msg__GPSRTK * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__GPSRTK__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__GPSRTK__Sequence__init(mavros_msgs__msg__GPSRTK__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSRTK * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__GPSRTK *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__GPSRTK), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__GPSRTK__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__GPSRTK__fini(&data[i - 1]);
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
mavros_msgs__msg__GPSRTK__Sequence__fini(mavros_msgs__msg__GPSRTK__Sequence * array)
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
      mavros_msgs__msg__GPSRTK__fini(&array->data[i]);
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

mavros_msgs__msg__GPSRTK__Sequence *
mavros_msgs__msg__GPSRTK__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__GPSRTK__Sequence * array = (mavros_msgs__msg__GPSRTK__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__GPSRTK__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__GPSRTK__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__GPSRTK__Sequence__destroy(mavros_msgs__msg__GPSRTK__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__GPSRTK__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__GPSRTK__Sequence__are_equal(const mavros_msgs__msg__GPSRTK__Sequence * lhs, const mavros_msgs__msg__GPSRTK__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__GPSRTK__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__GPSRTK__Sequence__copy(
  const mavros_msgs__msg__GPSRTK__Sequence * input,
  mavros_msgs__msg__GPSRTK__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__GPSRTK);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__GPSRTK * data =
      (mavros_msgs__msg__GPSRTK *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__GPSRTK__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__GPSRTK__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__GPSRTK__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
