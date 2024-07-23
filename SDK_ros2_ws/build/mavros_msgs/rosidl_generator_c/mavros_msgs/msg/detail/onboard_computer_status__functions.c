// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/OnboardComputerStatus.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/onboard_computer_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
mavros_msgs__msg__OnboardComputerStatus__init(mavros_msgs__msg__OnboardComputerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__OnboardComputerStatus__fini(msg);
    return false;
  }
  // component
  // uptime
  // type
  // cpu_cores
  // cpu_combined
  // gpu_cores
  // gpu_combined
  // temperature_board
  // temperature_core
  // fan_speed
  // ram_usage
  // ram_total
  // storage_type
  // storage_usage
  // storage_total
  // link_type
  // link_tx_rate
  // link_rx_rate
  // link_tx_max
  // link_rx_max
  return true;
}

void
mavros_msgs__msg__OnboardComputerStatus__fini(mavros_msgs__msg__OnboardComputerStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // component
  // uptime
  // type
  // cpu_cores
  // cpu_combined
  // gpu_cores
  // gpu_combined
  // temperature_board
  // temperature_core
  // fan_speed
  // ram_usage
  // ram_total
  // storage_type
  // storage_usage
  // storage_total
  // link_type
  // link_tx_rate
  // link_rx_rate
  // link_tx_max
  // link_rx_max
}

bool
mavros_msgs__msg__OnboardComputerStatus__are_equal(const mavros_msgs__msg__OnboardComputerStatus * lhs, const mavros_msgs__msg__OnboardComputerStatus * rhs)
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
  // component
  if (lhs->component != rhs->component) {
    return false;
  }
  // uptime
  if (lhs->uptime != rhs->uptime) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // cpu_cores
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->cpu_cores[i] != rhs->cpu_cores[i]) {
      return false;
    }
  }
  // cpu_combined
  for (size_t i = 0; i < 10; ++i) {
    if (lhs->cpu_combined[i] != rhs->cpu_combined[i]) {
      return false;
    }
  }
  // gpu_cores
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->gpu_cores[i] != rhs->gpu_cores[i]) {
      return false;
    }
  }
  // gpu_combined
  for (size_t i = 0; i < 10; ++i) {
    if (lhs->gpu_combined[i] != rhs->gpu_combined[i]) {
      return false;
    }
  }
  // temperature_board
  if (lhs->temperature_board != rhs->temperature_board) {
    return false;
  }
  // temperature_core
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->temperature_core[i] != rhs->temperature_core[i]) {
      return false;
    }
  }
  // fan_speed
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->fan_speed[i] != rhs->fan_speed[i]) {
      return false;
    }
  }
  // ram_usage
  if (lhs->ram_usage != rhs->ram_usage) {
    return false;
  }
  // ram_total
  if (lhs->ram_total != rhs->ram_total) {
    return false;
  }
  // storage_type
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->storage_type[i] != rhs->storage_type[i]) {
      return false;
    }
  }
  // storage_usage
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->storage_usage[i] != rhs->storage_usage[i]) {
      return false;
    }
  }
  // storage_total
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->storage_total[i] != rhs->storage_total[i]) {
      return false;
    }
  }
  // link_type
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->link_type[i] != rhs->link_type[i]) {
      return false;
    }
  }
  // link_tx_rate
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->link_tx_rate[i] != rhs->link_tx_rate[i]) {
      return false;
    }
  }
  // link_rx_rate
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->link_rx_rate[i] != rhs->link_rx_rate[i]) {
      return false;
    }
  }
  // link_tx_max
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->link_tx_max[i] != rhs->link_tx_max[i]) {
      return false;
    }
  }
  // link_rx_max
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->link_rx_max[i] != rhs->link_rx_max[i]) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__OnboardComputerStatus__copy(
  const mavros_msgs__msg__OnboardComputerStatus * input,
  mavros_msgs__msg__OnboardComputerStatus * output)
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
  // component
  output->component = input->component;
  // uptime
  output->uptime = input->uptime;
  // type
  output->type = input->type;
  // cpu_cores
  for (size_t i = 0; i < 8; ++i) {
    output->cpu_cores[i] = input->cpu_cores[i];
  }
  // cpu_combined
  for (size_t i = 0; i < 10; ++i) {
    output->cpu_combined[i] = input->cpu_combined[i];
  }
  // gpu_cores
  for (size_t i = 0; i < 4; ++i) {
    output->gpu_cores[i] = input->gpu_cores[i];
  }
  // gpu_combined
  for (size_t i = 0; i < 10; ++i) {
    output->gpu_combined[i] = input->gpu_combined[i];
  }
  // temperature_board
  output->temperature_board = input->temperature_board;
  // temperature_core
  for (size_t i = 0; i < 8; ++i) {
    output->temperature_core[i] = input->temperature_core[i];
  }
  // fan_speed
  for (size_t i = 0; i < 4; ++i) {
    output->fan_speed[i] = input->fan_speed[i];
  }
  // ram_usage
  output->ram_usage = input->ram_usage;
  // ram_total
  output->ram_total = input->ram_total;
  // storage_type
  for (size_t i = 0; i < 4; ++i) {
    output->storage_type[i] = input->storage_type[i];
  }
  // storage_usage
  for (size_t i = 0; i < 4; ++i) {
    output->storage_usage[i] = input->storage_usage[i];
  }
  // storage_total
  for (size_t i = 0; i < 4; ++i) {
    output->storage_total[i] = input->storage_total[i];
  }
  // link_type
  for (size_t i = 0; i < 6; ++i) {
    output->link_type[i] = input->link_type[i];
  }
  // link_tx_rate
  for (size_t i = 0; i < 6; ++i) {
    output->link_tx_rate[i] = input->link_tx_rate[i];
  }
  // link_rx_rate
  for (size_t i = 0; i < 6; ++i) {
    output->link_rx_rate[i] = input->link_rx_rate[i];
  }
  // link_tx_max
  for (size_t i = 0; i < 6; ++i) {
    output->link_tx_max[i] = input->link_tx_max[i];
  }
  // link_rx_max
  for (size_t i = 0; i < 6; ++i) {
    output->link_rx_max[i] = input->link_rx_max[i];
  }
  return true;
}

mavros_msgs__msg__OnboardComputerStatus *
mavros_msgs__msg__OnboardComputerStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OnboardComputerStatus * msg = (mavros_msgs__msg__OnboardComputerStatus *)allocator.allocate(sizeof(mavros_msgs__msg__OnboardComputerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__OnboardComputerStatus));
  bool success = mavros_msgs__msg__OnboardComputerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__OnboardComputerStatus__destroy(mavros_msgs__msg__OnboardComputerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__OnboardComputerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__OnboardComputerStatus__Sequence__init(mavros_msgs__msg__OnboardComputerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OnboardComputerStatus * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__OnboardComputerStatus *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__OnboardComputerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__OnboardComputerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__OnboardComputerStatus__fini(&data[i - 1]);
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
mavros_msgs__msg__OnboardComputerStatus__Sequence__fini(mavros_msgs__msg__OnboardComputerStatus__Sequence * array)
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
      mavros_msgs__msg__OnboardComputerStatus__fini(&array->data[i]);
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

mavros_msgs__msg__OnboardComputerStatus__Sequence *
mavros_msgs__msg__OnboardComputerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__OnboardComputerStatus__Sequence * array = (mavros_msgs__msg__OnboardComputerStatus__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__OnboardComputerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__OnboardComputerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__OnboardComputerStatus__Sequence__destroy(mavros_msgs__msg__OnboardComputerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__OnboardComputerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__OnboardComputerStatus__Sequence__are_equal(const mavros_msgs__msg__OnboardComputerStatus__Sequence * lhs, const mavros_msgs__msg__OnboardComputerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__OnboardComputerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__OnboardComputerStatus__Sequence__copy(
  const mavros_msgs__msg__OnboardComputerStatus__Sequence * input,
  mavros_msgs__msg__OnboardComputerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__OnboardComputerStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__OnboardComputerStatus * data =
      (mavros_msgs__msg__OnboardComputerStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__OnboardComputerStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__OnboardComputerStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__OnboardComputerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
