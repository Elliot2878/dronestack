// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ManualControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__STRUCT_H_

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

/// Struct defined in msg/ManualControl in the package mavros_msgs.
/**
  * Manual Control state
 */
typedef struct mavros_msgs__msg__ManualControl
{
  std_msgs__msg__Header header;
  float x;
  float y;
  float z;
  float r;
  uint16_t buttons;
} mavros_msgs__msg__ManualControl;

// Struct for a sequence of mavros_msgs__msg__ManualControl.
typedef struct mavros_msgs__msg__ManualControl__Sequence
{
  mavros_msgs__msg__ManualControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ManualControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__STRUCT_H_
