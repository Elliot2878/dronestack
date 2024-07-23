// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GlobalPositionTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GLOBAL_POSITION_TARGET__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GLOBAL_POSITION_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRAME_GLOBAL_INT'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__FRAME_GLOBAL_INT = 5
};

/// Constant 'FRAME_GLOBAL_REL_ALT'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__FRAME_GLOBAL_REL_ALT = 6
};

/// Constant 'FRAME_GLOBAL_TERRAIN_ALT'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__FRAME_GLOBAL_TERRAIN_ALT = 11
};

/// Constant 'IGNORE_LATITUDE'.
/**
  * Position ignore flags
 */
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_LATITUDE = 1
};

/// Constant 'IGNORE_LONGITUDE'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_LONGITUDE = 2
};

/// Constant 'IGNORE_ALTITUDE'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_ALTITUDE = 4
};

/// Constant 'IGNORE_VX'.
/**
  * Velocity vector ignore flags
 */
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_VX = 8
};

/// Constant 'IGNORE_VY'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_VY = 16
};

/// Constant 'IGNORE_VZ'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_VZ = 32
};

/// Constant 'IGNORE_AFX'.
/**
  * Acceleration/Force vector ignore flags
 */
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_AFX = 64
};

/// Constant 'IGNORE_AFY'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_AFY = 128
};

/// Constant 'IGNORE_AFZ'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_AFZ = 256
};

/// Constant 'FORCE'.
/**
  * Force in af vector flag
 */
enum
{
  mavros_msgs__msg__GlobalPositionTarget__FORCE = 512
};

/// Constant 'IGNORE_YAW'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_YAW = 1024
};

/// Constant 'IGNORE_YAW_RATE'.
enum
{
  mavros_msgs__msg__GlobalPositionTarget__IGNORE_YAW_RATE = 2048
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'velocity'
// Member 'acceleration_or_force'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/GlobalPositionTarget in the package mavros_msgs.
/**
  * Message for SET_POSITION_TARGET_GLOBAL_INT
  *
  * https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
  * Some complex system requires all feautures that mavlink
  * message provide. See issue #402, #415.
 */
typedef struct mavros_msgs__msg__GlobalPositionTarget
{
  std_msgs__msg__Header header;
  uint8_t coordinate_frame;
  uint16_t type_mask;
  double latitude;
  double longitude;
  /// in meters, AMSL or above terrain
  float altitude;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 acceleration_or_force;
  float yaw;
  float yaw_rate;
} mavros_msgs__msg__GlobalPositionTarget;

// Struct for a sequence of mavros_msgs__msg__GlobalPositionTarget.
typedef struct mavros_msgs__msg__GlobalPositionTarget__Sequence
{
  mavros_msgs__msg__GlobalPositionTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GlobalPositionTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GLOBAL_POSITION_TARGET__STRUCT_H_
