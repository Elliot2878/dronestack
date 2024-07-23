// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OpticalFlow.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__STRUCT_H_

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
// Member 'flow'
// Member 'flow_comp_m'
// Member 'flow_rate'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/OpticalFlow in the package mavros_msgs.
/**
  * OPTICAL_FLOW message data
 */
typedef struct mavros_msgs__msg__OpticalFlow
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 flow;
  geometry_msgs__msg__Vector3 flow_comp_m;
  uint8_t quality;
  float ground_distance;
  geometry_msgs__msg__Vector3 flow_rate;
} mavros_msgs__msg__OpticalFlow;

// Struct for a sequence of mavros_msgs__msg__OpticalFlow.
typedef struct mavros_msgs__msg__OpticalFlow__Sequence
{
  mavros_msgs__msg__OpticalFlow * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OpticalFlow__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__STRUCT_H_
