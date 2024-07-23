// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/WaypointReached.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__STRUCT_H_

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

/// Struct defined in msg/WaypointReached in the package mavros_msgs.
/**
  * That message represent MISSION_ITEM_REACHED
  *
  *   :wp_seq:    index number of reached waypoint
 */
typedef struct mavros_msgs__msg__WaypointReached
{
  std_msgs__msg__Header header;
  uint16_t wp_seq;
} mavros_msgs__msg__WaypointReached;

// Struct for a sequence of mavros_msgs__msg__WaypointReached.
typedef struct mavros_msgs__msg__WaypointReached__Sequence
{
  mavros_msgs__msg__WaypointReached * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__WaypointReached__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__STRUCT_H_
