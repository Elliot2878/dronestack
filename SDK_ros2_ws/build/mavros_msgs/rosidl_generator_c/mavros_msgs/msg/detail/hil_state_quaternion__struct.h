// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/HilStateQuaternion.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_STATE_QUATERNION__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__HIL_STATE_QUATERNION__STRUCT_H_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'angular_velocity'
// Member 'linear_acceleration'
// Member 'linear_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'geo'
#include "geographic_msgs/msg/detail/geo_point__struct.h"

/// Struct defined in msg/HilStateQuaternion in the package mavros_msgs.
/**
  * HilStateQuaternion.msg
  *
  * ROS representation of MAVLink HIL_STATE_QUATERNION
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION
 */
typedef struct mavros_msgs__msg__HilStateQuaternion
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Quaternion orientation;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Vector3 linear_acceleration;
  geometry_msgs__msg__Vector3 linear_velocity;
  geographic_msgs__msg__GeoPoint geo;
  float ind_airspeed;
  float true_airspeed;
} mavros_msgs__msg__HilStateQuaternion;

// Struct for a sequence of mavros_msgs__msg__HilStateQuaternion.
typedef struct mavros_msgs__msg__HilStateQuaternion__Sequence
{
  mavros_msgs__msg__HilStateQuaternion * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__HilStateQuaternion__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_STATE_QUATERNION__STRUCT_H_
