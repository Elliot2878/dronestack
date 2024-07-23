// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OpticalFlowRad.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__STRUCT_H_

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

/// Struct defined in msg/OpticalFlowRad in the package mavros_msgs.
/**
  * OPTICAL_FLOW_RAD message data
 */
typedef struct mavros_msgs__msg__OpticalFlowRad
{
  std_msgs__msg__Header header;
  uint32_t integration_time_us;
  float integrated_x;
  float integrated_y;
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
  int16_t temperature;
  uint8_t quality;
  uint32_t time_delta_distance_us;
  float distance;
} mavros_msgs__msg__OpticalFlowRad;

// Struct for a sequence of mavros_msgs__msg__OpticalFlowRad.
typedef struct mavros_msgs__msg__OpticalFlowRad__Sequence
{
  mavros_msgs__msg__OpticalFlowRad * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OpticalFlowRad__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__STRUCT_H_
