// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/HilActuatorControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__STRUCT_H_

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

/// Struct defined in msg/HilActuatorControls in the package mavros_msgs.
/**
  * HilActuatorControls.msg
  *
  * ROS representation of MAVLink HIL_ACTUATOR_CONTROLS
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
 */
typedef struct mavros_msgs__msg__HilActuatorControls
{
  std_msgs__msg__Header header;
  float controls[16];
  uint8_t mode;
  uint64_t flags;
} mavros_msgs__msg__HilActuatorControls;

// Struct for a sequence of mavros_msgs__msg__HilActuatorControls.
typedef struct mavros_msgs__msg__HilActuatorControls__Sequence
{
  mavros_msgs__msg__HilActuatorControls * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__HilActuatorControls__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__STRUCT_H_
