// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:srv/GimbalManagerSetRoi.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/srv/detail/gimbal_manager_set_roi__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
mavros_msgs__srv__GimbalManagerSetRoi_Request__init(mavros_msgs__srv__GimbalManagerSetRoi_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // gimbal_device_id
  // latitude
  // longitude
  // altitude
  // pitch_offset
  // roll_offset
  // yaw_offset
  // sysid
  return true;
}

void
mavros_msgs__srv__GimbalManagerSetRoi_Request__fini(mavros_msgs__srv__GimbalManagerSetRoi_Request * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // gimbal_device_id
  // latitude
  // longitude
  // altitude
  // pitch_offset
  // roll_offset
  // yaw_offset
  // sysid
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Request__are_equal(const mavros_msgs__srv__GimbalManagerSetRoi_Request * lhs, const mavros_msgs__srv__GimbalManagerSetRoi_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // gimbal_device_id
  if (lhs->gimbal_device_id != rhs->gimbal_device_id) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // pitch_offset
  if (lhs->pitch_offset != rhs->pitch_offset) {
    return false;
  }
  // roll_offset
  if (lhs->roll_offset != rhs->roll_offset) {
    return false;
  }
  // yaw_offset
  if (lhs->yaw_offset != rhs->yaw_offset) {
    return false;
  }
  // sysid
  if (lhs->sysid != rhs->sysid) {
    return false;
  }
  return true;
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Request__copy(
  const mavros_msgs__srv__GimbalManagerSetRoi_Request * input,
  mavros_msgs__srv__GimbalManagerSetRoi_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // gimbal_device_id
  output->gimbal_device_id = input->gimbal_device_id;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // pitch_offset
  output->pitch_offset = input->pitch_offset;
  // roll_offset
  output->roll_offset = input->roll_offset;
  // yaw_offset
  output->yaw_offset = input->yaw_offset;
  // sysid
  output->sysid = input->sysid;
  return true;
}

mavros_msgs__srv__GimbalManagerSetRoi_Request *
mavros_msgs__srv__GimbalManagerSetRoi_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__srv__GimbalManagerSetRoi_Request * msg = (mavros_msgs__srv__GimbalManagerSetRoi_Request *)allocator.allocate(sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Request));
  bool success = mavros_msgs__srv__GimbalManagerSetRoi_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__srv__GimbalManagerSetRoi_Request__destroy(mavros_msgs__srv__GimbalManagerSetRoi_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__srv__GimbalManagerSetRoi_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__init(mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__srv__GimbalManagerSetRoi_Request * data = NULL;

  if (size) {
    data = (mavros_msgs__srv__GimbalManagerSetRoi_Request *)allocator.zero_allocate(size, sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__srv__GimbalManagerSetRoi_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__srv__GimbalManagerSetRoi_Request__fini(&data[i - 1]);
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
mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__fini(mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * array)
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
      mavros_msgs__srv__GimbalManagerSetRoi_Request__fini(&array->data[i]);
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

mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence *
mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * array = (mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence *)allocator.allocate(sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__destroy(mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__are_equal(const mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * lhs, const mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__srv__GimbalManagerSetRoi_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence__copy(
  const mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * input,
  mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__srv__GimbalManagerSetRoi_Request * data =
      (mavros_msgs__srv__GimbalManagerSetRoi_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__srv__GimbalManagerSetRoi_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__srv__GimbalManagerSetRoi_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__srv__GimbalManagerSetRoi_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mavros_msgs__srv__GimbalManagerSetRoi_Response__init(mavros_msgs__srv__GimbalManagerSetRoi_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // result
  return true;
}

void
mavros_msgs__srv__GimbalManagerSetRoi_Response__fini(mavros_msgs__srv__GimbalManagerSetRoi_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // result
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Response__are_equal(const mavros_msgs__srv__GimbalManagerSetRoi_Response * lhs, const mavros_msgs__srv__GimbalManagerSetRoi_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Response__copy(
  const mavros_msgs__srv__GimbalManagerSetRoi_Response * input,
  mavros_msgs__srv__GimbalManagerSetRoi_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // result
  output->result = input->result;
  return true;
}

mavros_msgs__srv__GimbalManagerSetRoi_Response *
mavros_msgs__srv__GimbalManagerSetRoi_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__srv__GimbalManagerSetRoi_Response * msg = (mavros_msgs__srv__GimbalManagerSetRoi_Response *)allocator.allocate(sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Response));
  bool success = mavros_msgs__srv__GimbalManagerSetRoi_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__srv__GimbalManagerSetRoi_Response__destroy(mavros_msgs__srv__GimbalManagerSetRoi_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__srv__GimbalManagerSetRoi_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__init(mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__srv__GimbalManagerSetRoi_Response * data = NULL;

  if (size) {
    data = (mavros_msgs__srv__GimbalManagerSetRoi_Response *)allocator.zero_allocate(size, sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__srv__GimbalManagerSetRoi_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__srv__GimbalManagerSetRoi_Response__fini(&data[i - 1]);
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
mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__fini(mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * array)
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
      mavros_msgs__srv__GimbalManagerSetRoi_Response__fini(&array->data[i]);
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

mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence *
mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * array = (mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence *)allocator.allocate(sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__destroy(mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__are_equal(const mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * lhs, const mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__srv__GimbalManagerSetRoi_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence__copy(
  const mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * input,
  mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__srv__GimbalManagerSetRoi_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__srv__GimbalManagerSetRoi_Response * data =
      (mavros_msgs__srv__GimbalManagerSetRoi_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__srv__GimbalManagerSetRoi_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__srv__GimbalManagerSetRoi_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__srv__GimbalManagerSetRoi_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}