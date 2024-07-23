// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/RCIn.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RC_IN__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__RC_IN__STRUCT_H_

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
// Member 'channels'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RCIn in the package mavros_msgs.
/**
  * RAW RC input state
 */
typedef struct mavros_msgs__msg__RCIn
{
  std_msgs__msg__Header header;
  uint8_t rssi;
  rosidl_runtime_c__uint16__Sequence channels;
} mavros_msgs__msg__RCIn;

// Struct for a sequence of mavros_msgs__msg__RCIn.
typedef struct mavros_msgs__msg__RCIn__Sequence
{
  mavros_msgs__msg__RCIn * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__RCIn__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__RC_IN__STRUCT_H_
