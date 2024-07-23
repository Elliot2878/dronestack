// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/CompanionProcessStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAV_STATE_UNINIT'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_UNINIT = 0
};

/// Constant 'MAV_STATE_BOOT'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_BOOT = 1
};

/// Constant 'MAV_STATE_CALIBRATING'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_CALIBRATING = 2
};

/// Constant 'MAV_STATE_STANDBY'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_STANDBY = 3
};

/// Constant 'MAV_STATE_ACTIVE'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_ACTIVE = 4
};

/// Constant 'MAV_STATE_CRITICAL'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_CRITICAL = 5
};

/// Constant 'MAV_STATE_EMERGENCY'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_EMERGENCY = 6
};

/// Constant 'MAV_STATE_POWEROFF'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_POWEROFF = 7
};

/// Constant 'MAV_STATE_FLIGHT_TERMINATION'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_STATE_FLIGHT_TERMINATION = 8
};

/// Constant 'MAV_COMP_ID_OBSTACLE_AVOIDANCE'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196
};

/// Constant 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY'.
enum
{
  mavros_msgs__msg__CompanionProcessStatus__MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/CompanionProcessStatus in the package mavros_msgs.
/**
  * Mavros message: COMPANIONPROCESSSTATUS
 */
typedef struct mavros_msgs__msg__CompanionProcessStatus
{
  std_msgs__msg__Header header;
  /// See enum COMPANION_PROCESS_STATE
  uint8_t state;
  /// See enum MAV_COMPONENT
  uint8_t component;
} mavros_msgs__msg__CompanionProcessStatus;

// Struct for a sequence of mavros_msgs__msg__CompanionProcessStatus.
typedef struct mavros_msgs__msg__CompanionProcessStatus__Sequence
{
  mavros_msgs__msg__CompanionProcessStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__CompanionProcessStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__STRUCT_H_
