// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/WaypointList.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT_LIST__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'waypoints'
#include "mavros_msgs/msg/detail/waypoint__struct.h"

/// Struct defined in msg/WaypointList in the package mavros_msgs.
/**
  * WaypointList.msg
  *
  *  :current_seq:   seq nr of currently active waypoint
  *                  waypoints.is_current == True
  *
  *  :waypoints:     list of waypoints
 */
typedef struct mavros_msgs__msg__WaypointList
{
  uint16_t current_seq;
  mavros_msgs__msg__Waypoint__Sequence waypoints;
} mavros_msgs__msg__WaypointList;

// Struct for a sequence of mavros_msgs__msg__WaypointList.
typedef struct mavros_msgs__msg__WaypointList__Sequence
{
  mavros_msgs__msg__WaypointList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__WaypointList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT_LIST__STRUCT_H_
