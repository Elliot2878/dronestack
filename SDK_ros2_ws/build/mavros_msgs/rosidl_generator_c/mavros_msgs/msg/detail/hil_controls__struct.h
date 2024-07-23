// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/HilControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__STRUCT_H_

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

/// Struct defined in msg/HilControls in the package mavros_msgs.
/**
  * HilControls.msg
  *
  * ROS representation of MAVLink HIL_CONTROLS
  * (deprecated, use HIL_ACTUATOR_CONTROLS instead)
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#HIL_CONTROLS
 */
typedef struct mavros_msgs__msg__HilControls
{
  std_msgs__msg__Header header;
  float roll_ailerons;
  float pitch_elevator;
  float yaw_rudder;
  float throttle;
  float aux1;
  float aux2;
  float aux3;
  float aux4;
  uint8_t mode;
  uint8_t nav_mode;
} mavros_msgs__msg__HilControls;

// Struct for a sequence of mavros_msgs__msg__HilControls.
typedef struct mavros_msgs__msg__HilControls__Sequence
{
  mavros_msgs__msg__HilControls * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__HilControls__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__STRUCT_H_
