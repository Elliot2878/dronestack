// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/HilSensor.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__STRUCT_H_

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
// Member 'acc'
// Member 'gyro'
// Member 'mag'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/HilSensor in the package mavros_msgs.
/**
  * HilSensor.msg
  *
  * ROS representation of MAVLink HIL_SENSOR
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#HIL_SENSOR
 */
typedef struct mavros_msgs__msg__HilSensor
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 acc;
  geometry_msgs__msg__Vector3 gyro;
  geometry_msgs__msg__Vector3 mag;
  float abs_pressure;
  float diff_pressure;
  float pressure_alt;
  float temperature;
  uint32_t fields_updated;
} mavros_msgs__msg__HilSensor;

// Struct for a sequence of mavros_msgs__msg__HilSensor.
typedef struct mavros_msgs__msg__HilSensor__Sequence
{
  mavros_msgs__msg__HilSensor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__HilSensor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__STRUCT_H_
