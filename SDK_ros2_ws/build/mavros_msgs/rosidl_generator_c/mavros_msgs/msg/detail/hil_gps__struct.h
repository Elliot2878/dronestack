// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/HilGPS.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_GPS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__HIL_GPS__STRUCT_H_

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

/// Struct defined in msg/HilGPS in the package mavros_msgs.
/**
  * HilControls.msg
  *
  * ROS representation of MAVLink HIL_GPS
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#HIL_GPS
 */
typedef struct mavros_msgs__msg__HilGPS
{
  std_msgs__msg__Header header;
  uint8_t fix_type;
  geographic_msgs__msg__GeoPoint geo;
  uint16_t eph;
  uint16_t epv;
  uint16_t vel;
  int16_t vn;
  int16_t ve;
  int16_t vd;
  uint16_t cog;
  uint8_t satellites_visible;
} mavros_msgs__msg__HilGPS;

// Struct for a sequence of mavros_msgs__msg__HilGPS.
typedef struct mavros_msgs__msg__HilGPS__Sequence
{
  mavros_msgs__msg__HilGPS * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__HilGPS__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_GPS__STRUCT_H_
