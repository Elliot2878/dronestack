// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRAME_GLOBAL'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_GLOBAL = 0
};

/// Constant 'FRAME_LOCAL_NED'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_LOCAL_NED = 1
};

/// Constant 'FRAME_MISSION'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_MISSION = 2
};

/// Constant 'FRAME_GLOBAL_REL_ALT'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_GLOBAL_REL_ALT = 3
};

/// Constant 'FRAME_LOCAL_ENU'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_LOCAL_ENU = 4
};

/// Constant 'FRAME_GLOBAL_INT'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_GLOBAL_INT = 5
};

/// Constant 'FRAME_GLOBAL_RELATIVE_ALT_INT'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_GLOBAL_RELATIVE_ALT_INT = 6
};

/// Constant 'FRAME_LOCAL_OFFSET_NED'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_LOCAL_OFFSET_NED = 7
};

/// Constant 'FRAME_BODY_NED'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_BODY_NED = 8
};

/// Constant 'FRAME_BODY_OFFSET_NED'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_BODY_OFFSET_NED = 9
};

/// Constant 'FRAME_GLOBAL_TERRAIN_ALT'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_GLOBAL_TERRAIN_ALT = 10
};

/// Constant 'FRAME_GLOBAL_TERRAIN_ALT_INT'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_GLOBAL_TERRAIN_ALT_INT = 11
};

/// Constant 'FRAME_BODY_FRD'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_BODY_FRD = 12
};

/// Constant 'FRAME_RESERVED_13'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_13 = 13
};

/// Constant 'FRAME_RESERVED_14'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_14 = 14
};

/// Constant 'FRAME_RESERVED_15'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_15 = 15
};

/// Constant 'FRAME_RESERVED_16'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_16 = 16
};

/// Constant 'FRAME_RESERVED_17'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_17 = 17
};

/// Constant 'FRAME_RESERVED_18'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_18 = 18
};

/// Constant 'FRAME_RESERVED_19'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_RESERVED_19 = 19
};

/// Constant 'FRAME_LOCAL_FRD'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_LOCAL_FRD = 20
};

/// Constant 'FRAME_LOCAL_FLU'.
enum
{
  mavros_msgs__msg__Waypoint__FRAME_LOCAL_FLU = 21
};

/// Struct defined in msg/Waypoint in the package mavros_msgs.
/**
  * Waypoint.msg
  *
  * ROS representation of MAVLink MISSION_ITEM
  * See mavlink documentation
 */
typedef struct mavros_msgs__msg__Waypoint
{
  /// see enum MAV_FRAME
  uint8_t frame;
  /// see enum MAV_CMD and CommandCode.msg
  uint16_t command;
  bool is_current;
  bool autocontinue;
  /// meaning of this params described in enum MAV_CMD
  float param1;
  float param2;
  float param3;
  float param4;
  double x_lat;
  double y_long;
  double z_alt;
} mavros_msgs__msg__Waypoint;

// Struct for a sequence of mavros_msgs__msg__Waypoint.
typedef struct mavros_msgs__msg__Waypoint__Sequence
{
  mavros_msgs__msg__Waypoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__Waypoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_H_
