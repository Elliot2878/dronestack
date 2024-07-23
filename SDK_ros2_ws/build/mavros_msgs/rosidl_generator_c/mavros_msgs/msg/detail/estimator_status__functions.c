// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/estimator_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__EstimatorStatus__init(mavros_msgs__msg__EstimatorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__EstimatorStatus__fini(msg);
    return false;
  }
  // attitude_status_flag
  // velocity_horiz_status_flag
  // velocity_vert_status_flag
  // pos_horiz_rel_status_flag
  // pos_horiz_abs_status_flag
  // pos_vert_abs_status_flag
  // pos_vert_agl_status_flag
  // const_pos_mode_status_flag
  // pred_pos_horiz_rel_status_flag
  // pred_pos_horiz_abs_status_flag
  // gps_glitch_status_flag
  // accel_error_status_flag
  return true;
}

void
mavros_msgs__msg__EstimatorStatus__fini(mavros_msgs__msg__EstimatorStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // attitude_status_flag
  // velocity_horiz_status_flag
  // velocity_vert_status_flag
  // pos_horiz_rel_status_flag
  // pos_horiz_abs_status_flag
  // pos_vert_abs_status_flag
  // pos_vert_agl_status_flag
  // const_pos_mode_status_flag
  // pred_pos_horiz_rel_status_flag
  // pred_pos_horiz_abs_status_flag
  // gps_glitch_status_flag
  // accel_error_status_flag
}

bool
mavros_msgs__msg__EstimatorStatus__are_equal(const mavros_msgs__msg__EstimatorStatus * lhs, const mavros_msgs__msg__EstimatorStatus * rhs)
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
  // attitude_status_flag
  if (lhs->attitude_status_flag != rhs->attitude_status_flag) {
    return false;
  }
  // velocity_horiz_status_flag
  if (lhs->velocity_horiz_status_flag != rhs->velocity_horiz_status_flag) {
    return false;
  }
  // velocity_vert_status_flag
  if (lhs->velocity_vert_status_flag != rhs->velocity_vert_status_flag) {
    return false;
  }
  // pos_horiz_rel_status_flag
  if (lhs->pos_horiz_rel_status_flag != rhs->pos_horiz_rel_status_flag) {
    return false;
  }
  // pos_horiz_abs_status_flag
  if (lhs->pos_horiz_abs_status_flag != rhs->pos_horiz_abs_status_flag) {
    return false;
  }
  // pos_vert_abs_status_flag
  if (lhs->pos_vert_abs_status_flag != rhs->pos_vert_abs_status_flag) {
    return false;
  }
  // pos_vert_agl_status_flag
  if (lhs->pos_vert_agl_status_flag != rhs->pos_vert_agl_status_flag) {
    return false;
  }
  // const_pos_mode_status_flag
  if (lhs->const_pos_mode_status_flag != rhs->const_pos_mode_status_flag) {
    return false;
  }
  // pred_pos_horiz_rel_status_flag
  if (lhs->pred_pos_horiz_rel_status_flag != rhs->pred_pos_horiz_rel_status_flag) {
    return false;
  }
  // pred_pos_horiz_abs_status_flag
  if (lhs->pred_pos_horiz_abs_status_flag != rhs->pred_pos_horiz_abs_status_flag) {
    return false;
  }
  // gps_glitch_status_flag
  if (lhs->gps_glitch_status_flag != rhs->gps_glitch_status_flag) {
    return false;
  }
  // accel_error_status_flag
  if (lhs->accel_error_status_flag != rhs->accel_error_status_flag) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__EstimatorStatus__copy(
  const mavros_msgs__msg__EstimatorStatus * input,
  mavros_msgs__msg__EstimatorStatus * output)
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
  // attitude_status_flag
  output->attitude_status_flag = input->attitude_status_flag;
  // velocity_horiz_status_flag
  output->velocity_horiz_status_flag = input->velocity_horiz_status_flag;
  // velocity_vert_status_flag
  output->velocity_vert_status_flag = input->velocity_vert_status_flag;
  // pos_horiz_rel_status_flag
  output->pos_horiz_rel_status_flag = input->pos_horiz_rel_status_flag;
  // pos_horiz_abs_status_flag
  output->pos_horiz_abs_status_flag = input->pos_horiz_abs_status_flag;
  // pos_vert_abs_status_flag
  output->pos_vert_abs_status_flag = input->pos_vert_abs_status_flag;
  // pos_vert_agl_status_flag
  output->pos_vert_agl_status_flag = input->pos_vert_agl_status_flag;
  // const_pos_mode_status_flag
  output->const_pos_mode_status_flag = input->const_pos_mode_status_flag;
  // pred_pos_horiz_rel_status_flag
  output->pred_pos_horiz_rel_status_flag = input->pred_pos_horiz_rel_status_flag;
  // pred_pos_horiz_abs_status_flag
  output->pred_pos_horiz_abs_status_flag = input->pred_pos_horiz_abs_status_flag;
  // gps_glitch_status_flag
  output->gps_glitch_status_flag = input->gps_glitch_status_flag;
  // accel_error_status_flag
  output->accel_error_status_flag = input->accel_error_status_flag;
  return true;
}

mavros_msgs__msg__EstimatorStatus *
mavros_msgs__msg__EstimatorStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__EstimatorStatus * msg = (mavros_msgs__msg__EstimatorStatus *)allocator.allocate(sizeof(mavros_msgs__msg__EstimatorStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__EstimatorStatus));
  bool success = mavros_msgs__msg__EstimatorStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__EstimatorStatus__destroy(mavros_msgs__msg__EstimatorStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__EstimatorStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__EstimatorStatus__Sequence__init(mavros_msgs__msg__EstimatorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__EstimatorStatus * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__EstimatorStatus *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__EstimatorStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__EstimatorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__EstimatorStatus__fini(&data[i - 1]);
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
mavros_msgs__msg__EstimatorStatus__Sequence__fini(mavros_msgs__msg__EstimatorStatus__Sequence * array)
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
      mavros_msgs__msg__EstimatorStatus__fini(&array->data[i]);
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

mavros_msgs__msg__EstimatorStatus__Sequence *
mavros_msgs__msg__EstimatorStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__EstimatorStatus__Sequence * array = (mavros_msgs__msg__EstimatorStatus__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__EstimatorStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__EstimatorStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__EstimatorStatus__Sequence__destroy(mavros_msgs__msg__EstimatorStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__EstimatorStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__EstimatorStatus__Sequence__are_equal(const mavros_msgs__msg__EstimatorStatus__Sequence * lhs, const mavros_msgs__msg__EstimatorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__EstimatorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__EstimatorStatus__Sequence__copy(
  const mavros_msgs__msg__EstimatorStatus__Sequence * input,
  mavros_msgs__msg__EstimatorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__EstimatorStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__EstimatorStatus * data =
      (mavros_msgs__msg__EstimatorStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__EstimatorStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__EstimatorStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__EstimatorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
