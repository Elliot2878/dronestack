// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/SysStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SysStatus in the package mavros_msgs.
typedef struct mavros_msgs__msg__SysStatus
{
  std_msgs__msg__Header header;
  uint32_t sensors_present;
  uint32_t sensors_enabled;
  uint32_t sensors_health;
  uint16_t load;
  uint16_t voltage_battery;
  int16_t current_battery;
  int8_t battery_remaining;
  uint16_t drop_rate_comm;
  uint16_t errors_comm;
  uint16_t errors_count1;
  uint16_t errors_count2;
  uint16_t errors_count3;
  uint16_t errors_count4;
} mavros_msgs__msg__SysStatus;

// Struct for a sequence of mavros_msgs__msg__SysStatus.
typedef struct mavros_msgs__msg__SysStatus__Sequence
{
  mavros_msgs__msg__SysStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__SysStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__STRUCT_H_
