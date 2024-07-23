// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ExtendedState.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'VTOL_STATE_UNDEFINED'.
enum
{
  mavros_msgs__msg__ExtendedState__VTOL_STATE_UNDEFINED = 0
};

/// Constant 'VTOL_STATE_TRANSITION_TO_FW'.
enum
{
  mavros_msgs__msg__ExtendedState__VTOL_STATE_TRANSITION_TO_FW = 1
};

/// Constant 'VTOL_STATE_TRANSITION_TO_MC'.
enum
{
  mavros_msgs__msg__ExtendedState__VTOL_STATE_TRANSITION_TO_MC = 2
};

/// Constant 'VTOL_STATE_MC'.
enum
{
  mavros_msgs__msg__ExtendedState__VTOL_STATE_MC = 3
};

/// Constant 'VTOL_STATE_FW'.
enum
{
  mavros_msgs__msg__ExtendedState__VTOL_STATE_FW = 4
};

/// Constant 'LANDED_STATE_UNDEFINED'.
enum
{
  mavros_msgs__msg__ExtendedState__LANDED_STATE_UNDEFINED = 0
};

/// Constant 'LANDED_STATE_ON_GROUND'.
enum
{
  mavros_msgs__msg__ExtendedState__LANDED_STATE_ON_GROUND = 1
};

/// Constant 'LANDED_STATE_IN_AIR'.
enum
{
  mavros_msgs__msg__ExtendedState__LANDED_STATE_IN_AIR = 2
};

/// Constant 'LANDED_STATE_TAKEOFF'.
enum
{
  mavros_msgs__msg__ExtendedState__LANDED_STATE_TAKEOFF = 3
};

/// Constant 'LANDED_STATE_LANDING'.
enum
{
  mavros_msgs__msg__ExtendedState__LANDED_STATE_LANDING = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ExtendedState in the package mavros_msgs.
/**
  * Extended autopilot state
  *
  * https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE
 */
typedef struct mavros_msgs__msg__ExtendedState
{
  std_msgs__msg__Header header;
  uint8_t vtol_state;
  uint8_t landed_state;
} mavros_msgs__msg__ExtendedState;

// Struct for a sequence of mavros_msgs__msg__ExtendedState.
typedef struct mavros_msgs__msg__ExtendedState__Sequence
{
  mavros_msgs__msg__ExtendedState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ExtendedState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__STRUCT_H_
