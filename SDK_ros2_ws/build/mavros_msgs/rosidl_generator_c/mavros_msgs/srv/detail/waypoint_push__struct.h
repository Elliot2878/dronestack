// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/WaypointPush.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__STRUCT_H_

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

/// Struct defined in srv/WaypointPush in the package mavros_msgs.
typedef struct mavros_msgs__srv__WaypointPush_Request
{
  uint16_t start_index;
  mavros_msgs__msg__Waypoint__Sequence waypoints;
} mavros_msgs__srv__WaypointPush_Request;

// Struct for a sequence of mavros_msgs__srv__WaypointPush_Request.
typedef struct mavros_msgs__srv__WaypointPush_Request__Sequence
{
  mavros_msgs__srv__WaypointPush_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__WaypointPush_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/WaypointPush in the package mavros_msgs.
typedef struct mavros_msgs__srv__WaypointPush_Response
{
  bool success;
  uint32_t wp_transfered;
} mavros_msgs__srv__WaypointPush_Response;

// Struct for a sequence of mavros_msgs__srv__WaypointPush_Response.
typedef struct mavros_msgs__srv__WaypointPush_Response__Sequence
{
  mavros_msgs__srv__WaypointPush_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__WaypointPush_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__STRUCT_H_
