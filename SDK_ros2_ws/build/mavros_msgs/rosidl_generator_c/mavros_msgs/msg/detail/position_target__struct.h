// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/PositionTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__POSITION_TARGET__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__POSITION_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRAME_LOCAL_NED'.
enum
{
  mavros_msgs__msg__PositionTarget__FRAME_LOCAL_NED = 1
};

/// Constant 'FRAME_LOCAL_OFFSET_NED'.
enum
{
  mavros_msgs__msg__PositionTarget__FRAME_LOCAL_OFFSET_NED = 7
};

/// Constant 'FRAME_BODY_NED'.
enum
{
  mavros_msgs__msg__PositionTarget__FRAME_BODY_NED = 8
};

/// Constant 'FRAME_BODY_OFFSET_NED'.
enum
{
  mavros_msgs__msg__PositionTarget__FRAME_BODY_OFFSET_NED = 9
};

/// Constant 'IGNORE_PX'.
/**
  * Position ignore flags
 */
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_PX = 1
};

/// Constant 'IGNORE_PY'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_PY = 2
};

/// Constant 'IGNORE_PZ'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_PZ = 4
};

/// Constant 'IGNORE_VX'.
/**
  * Velocity vector ignore flags
 */
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_VX = 8
};

/// Constant 'IGNORE_VY'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_VY = 16
};

/// Constant 'IGNORE_VZ'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_VZ = 32
};

/// Constant 'IGNORE_AFX'.
/**
  * Acceleration/Force vector ignore flags
 */
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_AFX = 64
};

/// Constant 'IGNORE_AFY'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_AFY = 128
};

/// Constant 'IGNORE_AFZ'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_AFZ = 256
};

/// Constant 'FORCE'.
/**
  * Force in af vector flag
 */
enum
{
  mavros_msgs__msg__PositionTarget__FORCE = 512
};

/// Constant 'IGNORE_YAW'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_YAW = 1024
};

/// Constant 'IGNORE_YAW_RATE'.
enum
{
  mavros_msgs__msg__PositionTarget__IGNORE_YAW_RATE = 2048
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'acceleration_or_force'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/PositionTarget in the package mavros_msgs.
/**
  * Message for SET_POSITION_TARGET_LOCAL_NED
  *
  * Some complex system requires all feautures that mavlink
  * message provide. See issue #402.
 */
typedef struct mavros_msgs__msg__PositionTarget
{
  std_msgs__msg__Header header;
  uint8_t coordinate_frame;
  uint16_t type_mask;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 acceleration_or_force;
  float yaw;
  float yaw_rate;
} mavros_msgs__msg__PositionTarget;

// Struct for a sequence of mavros_msgs__msg__PositionTarget.
typedef struct mavros_msgs__msg__PositionTarget__Sequence
{
  mavros_msgs__msg__PositionTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__PositionTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__POSITION_TARGET__STRUCT_H_
