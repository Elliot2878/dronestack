// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/Vibration.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VIBRATION__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__VIBRATION__STRUCT_H_

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
// Member 'vibration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Vibration in the package mavros_msgs.
/**
  * VIBRATION message data
  * @description: Vibration levels and accelerometer clipping
 */
typedef struct mavros_msgs__msg__Vibration
{
  std_msgs__msg__Header header;
  /// 3-axis vibration levels
  geometry_msgs__msg__Vector3 vibration;
  /// Accelerometers clipping
  float clipping[3];
} mavros_msgs__msg__Vibration;

// Struct for a sequence of mavros_msgs__msg__Vibration.
typedef struct mavros_msgs__msg__Vibration__Sequence
{
  mavros_msgs__msg__Vibration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__Vibration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__VIBRATION__STRUCT_H_
