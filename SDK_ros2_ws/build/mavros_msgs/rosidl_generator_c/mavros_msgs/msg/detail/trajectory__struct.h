// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS'.
enum
{
  mavros_msgs__msg__Trajectory__MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0
};

/// Constant 'MAV_TRAJECTORY_REPRESENTATION_BEZIER'.
enum
{
  mavros_msgs__msg__Trajectory__MAV_TRAJECTORY_REPRESENTATION_BEZIER = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'point_1'
// Member 'point_2'
// Member 'point_3'
// Member 'point_4'
// Member 'point_5'
#include "mavros_msgs/msg/detail/position_target__struct.h"

/// Struct defined in msg/Trajectory in the package mavros_msgs.
/**
  * MAVLink message: TRAJECTORY
  * https://mavlink.io/en/messages/common.html#TRAJECTORY
 */
typedef struct mavros_msgs__msg__Trajectory
{
  std_msgs__msg__Header header;
  /// See enum MAV_TRAJECTORY_REPRESENTATION.
  uint8_t type;
  mavros_msgs__msg__PositionTarget point_1;
  mavros_msgs__msg__PositionTarget point_2;
  mavros_msgs__msg__PositionTarget point_3;
  mavros_msgs__msg__PositionTarget point_4;
  mavros_msgs__msg__PositionTarget point_5;
  /// States if respective point is valid.
  uint8_t point_valid[5];
  /// MAV_CMD associated with each point. UINT16_MAX if unused.
  uint16_t command[5];
  /// if type MAV_TRAJECTORY_REPRESENTATION_BEZIER, it represents the time horizon for each point, otherwise set to NaN
  float time_horizon[5];
} mavros_msgs__msg__Trajectory;

// Struct for a sequence of mavros_msgs__msg__Trajectory.
typedef struct mavros_msgs__msg__Trajectory__Sequence
{
  mavros_msgs__msg__Trajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__Trajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
