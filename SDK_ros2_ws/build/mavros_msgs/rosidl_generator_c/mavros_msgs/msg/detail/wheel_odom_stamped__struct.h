// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/WheelOdomStamped.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WHEEL_ODOM_STAMPED__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__WHEEL_ODOM_STAMPED__STRUCT_H_

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

/// Struct defined in msg/WheelOdomStamped in the package mavros_msgs.
/**
  * Stamped wheel odometry message
  *
  * For streaming timestamped data from FCU wheel encoders (RPM or WHEEL_DISTANCE).
 */
typedef struct mavros_msgs__msg__WheelOdomStamped
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence data;
} mavros_msgs__msg__WheelOdomStamped;

// Struct for a sequence of mavros_msgs__msg__WheelOdomStamped.
typedef struct mavros_msgs__msg__WheelOdomStamped__Sequence
{
  mavros_msgs__msg__WheelOdomStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__WheelOdomStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__WHEEL_ODOM_STAMPED__STRUCT_H_
