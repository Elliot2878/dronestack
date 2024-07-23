// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/NavControllerOutput.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__STRUCT_H_

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

/// Struct defined in msg/NavControllerOutput in the package mavros_msgs.
/**
  * https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
 */
typedef struct mavros_msgs__msg__NavControllerOutput
{
  std_msgs__msg__Header header;
  /// Current desired roll
  float nav_roll;
  /// Current desired pitch
  float nav_pitch;
  /// Current desired heading
  int16_t nav_bearing;
  /// Bearing to current waypoint/target
  int16_t target_bearing;
  /// Distance to active waypoint
  uint16_t wp_dist;
  /// Current altitude error
  float alt_error;
  /// Current airspeed error
  float aspd_error;
  /// Current crosstrack error on x-y plane
  float xtrack_error;
} mavros_msgs__msg__NavControllerOutput;

// Struct for a sequence of mavros_msgs__msg__NavControllerOutput.
typedef struct mavros_msgs__msg__NavControllerOutput__Sequence
{
  mavros_msgs__msg__NavControllerOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__NavControllerOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__STRUCT_H_
