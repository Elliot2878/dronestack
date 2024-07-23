// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/VehicleInfo.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/vehicle_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `mode`
// Member `flight_custom_version`
#include "rosidl_runtime_c/string_functions.h"

bool
mavros_msgs__msg__VehicleInfo__init(mavros_msgs__msg__VehicleInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__VehicleInfo__fini(msg);
    return false;
  }
  // available_info
  // sysid
  // compid
  // autopilot
  // type
  // system_status
  // base_mode
  // custom_mode
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    mavros_msgs__msg__VehicleInfo__fini(msg);
    return false;
  }
  // mode_id
  // capabilities
  // flight_sw_version
  // middleware_sw_version
  // os_sw_version
  // board_version
  // flight_custom_version
  if (!rosidl_runtime_c__String__init(&msg->flight_custom_version)) {
    mavros_msgs__msg__VehicleInfo__fini(msg);
    return false;
  }
  // vendor_id
  // product_id
  // uid
  return true;
}

void
mavros_msgs__msg__VehicleInfo__fini(mavros_msgs__msg__VehicleInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // available_info
  // sysid
  // compid
  // autopilot
  // type
  // system_status
  // base_mode
  // custom_mode
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
  // mode_id
  // capabilities
  // flight_sw_version
  // middleware_sw_version
  // os_sw_version
  // board_version
  // flight_custom_version
  rosidl_runtime_c__String__fini(&msg->flight_custom_version);
  // vendor_id
  // product_id
  // uid
}

bool
mavros_msgs__msg__VehicleInfo__are_equal(const mavros_msgs__msg__VehicleInfo * lhs, const mavros_msgs__msg__VehicleInfo * rhs)
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
  // available_info
  if (lhs->available_info != rhs->available_info) {
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
  // autopilot
  if (lhs->autopilot != rhs->autopilot) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // system_status
  if (lhs->system_status != rhs->system_status) {
    return false;
  }
  // base_mode
  if (lhs->base_mode != rhs->base_mode) {
    return false;
  }
  // custom_mode
  if (lhs->custom_mode != rhs->custom_mode) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // mode_id
  if (lhs->mode_id != rhs->mode_id) {
    return false;
  }
  // capabilities
  if (lhs->capabilities != rhs->capabilities) {
    return false;
  }
  // flight_sw_version
  if (lhs->flight_sw_version != rhs->flight_sw_version) {
    return false;
  }
  // middleware_sw_version
  if (lhs->middleware_sw_version != rhs->middleware_sw_version) {
    return false;
  }
  // os_sw_version
  if (lhs->os_sw_version != rhs->os_sw_version) {
    return false;
  }
  // board_version
  if (lhs->board_version != rhs->board_version) {
    return false;
  }
  // flight_custom_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->flight_custom_version), &(rhs->flight_custom_version)))
  {
    return false;
  }
  // vendor_id
  if (lhs->vendor_id != rhs->vendor_id) {
    return false;
  }
  // product_id
  if (lhs->product_id != rhs->product_id) {
    return false;
  }
  // uid
  if (lhs->uid != rhs->uid) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__VehicleInfo__copy(
  const mavros_msgs__msg__VehicleInfo * input,
  mavros_msgs__msg__VehicleInfo * output)
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
  // available_info
  output->available_info = input->available_info;
  // sysid
  output->sysid = input->sysid;
  // compid
  output->compid = input->compid;
  // autopilot
  output->autopilot = input->autopilot;
  // type
  output->type = input->type;
  // system_status
  output->system_status = input->system_status;
  // base_mode
  output->base_mode = input->base_mode;
  // custom_mode
  output->custom_mode = input->custom_mode;
  // mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // mode_id
  output->mode_id = input->mode_id;
  // capabilities
  output->capabilities = input->capabilities;
  // flight_sw_version
  output->flight_sw_version = input->flight_sw_version;
  // middleware_sw_version
  output->middleware_sw_version = input->middleware_sw_version;
  // os_sw_version
  output->os_sw_version = input->os_sw_version;
  // board_version
  output->board_version = input->board_version;
  // flight_custom_version
  if (!rosidl_runtime_c__String__copy(
      &(input->flight_custom_version), &(output->flight_custom_version)))
  {
    return false;
  }
  // vendor_id
  output->vendor_id = input->vendor_id;
  // product_id
  output->product_id = input->product_id;
  // uid
  output->uid = input->uid;
  return true;
}

mavros_msgs__msg__VehicleInfo *
mavros_msgs__msg__VehicleInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__VehicleInfo * msg = (mavros_msgs__msg__VehicleInfo *)allocator.allocate(sizeof(mavros_msgs__msg__VehicleInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__VehicleInfo));
  bool success = mavros_msgs__msg__VehicleInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__VehicleInfo__destroy(mavros_msgs__msg__VehicleInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__VehicleInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__VehicleInfo__Sequence__init(mavros_msgs__msg__VehicleInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__VehicleInfo * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__VehicleInfo *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__VehicleInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__VehicleInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__VehicleInfo__fini(&data[i - 1]);
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
mavros_msgs__msg__VehicleInfo__Sequence__fini(mavros_msgs__msg__VehicleInfo__Sequence * array)
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
      mavros_msgs__msg__VehicleInfo__fini(&array->data[i]);
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

mavros_msgs__msg__VehicleInfo__Sequence *
mavros_msgs__msg__VehicleInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__VehicleInfo__Sequence * array = (mavros_msgs__msg__VehicleInfo__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__VehicleInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__VehicleInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__VehicleInfo__Sequence__destroy(mavros_msgs__msg__VehicleInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__VehicleInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__VehicleInfo__Sequence__are_equal(const mavros_msgs__msg__VehicleInfo__Sequence * lhs, const mavros_msgs__msg__VehicleInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__VehicleInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__VehicleInfo__Sequence__copy(
  const mavros_msgs__msg__VehicleInfo__Sequence * input,
  mavros_msgs__msg__VehicleInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__VehicleInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__VehicleInfo * data =
      (mavros_msgs__msg__VehicleInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__VehicleInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__VehicleInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__VehicleInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
