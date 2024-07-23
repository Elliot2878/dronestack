// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/RCOut.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RC_OUT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__RC_OUT__STRUCT_H_

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

/// Struct defined in msg/RCOut in the package mavros_msgs.
/**
  * RAW Servo out state
 */
typedef struct mavros_msgs__msg__RCOut
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__uint16__Sequence channels;
} mavros_msgs__msg__RCOut;

// Struct for a sequence of mavros_msgs__msg__RCOut.
typedef struct mavros_msgs__msg__RCOut__Sequence
{
  mavros_msgs__msg__RCOut * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__RCOut__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__RC_OUT__STRUCT_H_
