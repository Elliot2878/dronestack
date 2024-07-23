// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/HomePosition.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__STRUCT_H_

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
// Member 'geo'
#include "geographic_msgs/msg/detail/geo_point__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'approach'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/HomePosition in the package mavros_msgs.
/**
  * MAVLink message: HOME_POSITION
  * https://mavlink.io/en/messages/common.html#HOME_POSITION
 */
typedef struct mavros_msgs__msg__HomePosition
{
  std_msgs__msg__Header header;
  /// geodetic coordinates in WGS-84 datum
  geographic_msgs__msg__GeoPoint geo;
  /// local position
  geometry_msgs__msg__Point position;
  /// XXX: verify field name (q)
  geometry_msgs__msg__Quaternion orientation;
  /// position of the end of approach vector
  geometry_msgs__msg__Vector3 approach;
} mavros_msgs__msg__HomePosition;

// Struct for a sequence of mavros_msgs__msg__HomePosition.
typedef struct mavros_msgs__msg__HomePosition__Sequence
{
  mavros_msgs__msg__HomePosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__HomePosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__STRUCT_H_
