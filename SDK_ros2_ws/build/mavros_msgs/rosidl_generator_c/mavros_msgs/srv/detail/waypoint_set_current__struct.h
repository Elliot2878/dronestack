// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/WaypointSetCurrent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/WaypointSetCurrent in the package mavros_msgs.
typedef struct mavros_msgs__srv__WaypointSetCurrent_Request
{
  uint16_t wp_seq;
} mavros_msgs__srv__WaypointSetCurrent_Request;

// Struct for a sequence of mavros_msgs__srv__WaypointSetCurrent_Request.
typedef struct mavros_msgs__srv__WaypointSetCurrent_Request__Sequence
{
  mavros_msgs__srv__WaypointSetCurrent_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__WaypointSetCurrent_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/WaypointSetCurrent in the package mavros_msgs.
typedef struct mavros_msgs__srv__WaypointSetCurrent_Response
{
  bool success;
} mavros_msgs__srv__WaypointSetCurrent_Response;

// Struct for a sequence of mavros_msgs__srv__WaypointSetCurrent_Response.
typedef struct mavros_msgs__srv__WaypointSetCurrent_Response__Sequence
{
  mavros_msgs__srv__WaypointSetCurrent_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__WaypointSetCurrent_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__STRUCT_H_
