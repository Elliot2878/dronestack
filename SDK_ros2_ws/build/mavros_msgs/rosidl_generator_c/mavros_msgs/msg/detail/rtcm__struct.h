// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/RTCM.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RTCM__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__RTCM__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RTCM in the package mavros_msgs.
/**
  * RTCM message for the gps_rtk plugin
  * The gps_rtk plugin will fragment the data if necessary and
  * forward it to the FCU via Mavlink through the available link.
  * data should be <= 4*180, higher will be discarded.
 */
typedef struct mavros_msgs__msg__RTCM
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__uint8__Sequence data;
} mavros_msgs__msg__RTCM;

// Struct for a sequence of mavros_msgs__msg__RTCM.
typedef struct mavros_msgs__msg__RTCM__Sequence
{
  mavros_msgs__msg__RTCM * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__RTCM__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__RTCM__STRUCT_H_
