// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GimbalManagerStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GIMBAL_MANAGER_FLAGS_RETRACT'.
/**
  * GIMBAL_MANAGER_FLAGS
  *  Based on GIMBAL_DEVICE_FLAGS_RETRACT
 */
enum
{
  mavros_msgs__msg__GimbalManagerStatus__GIMBAL_MANAGER_FLAGS_RETRACT = 1ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_NEUTRAL'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
 */
enum
{
  mavros_msgs__msg__GimbalManagerStatus__GIMBAL_MANAGER_FLAGS_NEUTRAL = 2ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_ROLL_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalManagerStatus__GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_PITCH_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalManagerStatus__GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_YAW_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalManagerStatus__GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GimbalManagerStatus in the package mavros_msgs.
/**
  * MAVLink message: GIMBAL_MANAGER_STATUS
  * https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS
 */
typedef struct mavros_msgs__msg__GimbalManagerStatus
{
  std_msgs__msg__Header header;
  /// High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
  uint32_t flags;
  /// Gimbal device ID that this gimbal manager is responsible for.
  uint8_t gimbal_device_id;
  /// System ID of MAVLink component with primary control, 0 for none.
  uint8_t sysid_primary;
  /// Component ID of MAVLink component with primary control, 0 for none.
  uint8_t compid_primary;
  /// System ID of MAVLink component with secondary control, 0 for none.
  uint8_t sysid_secondary;
  /// Component ID of MAVLink component with secondary control, 0 for none.
  uint8_t compid_secondary;
} mavros_msgs__msg__GimbalManagerStatus;

// Struct for a sequence of mavros_msgs__msg__GimbalManagerStatus.
typedef struct mavros_msgs__msg__GimbalManagerStatus__Sequence
{
  mavros_msgs__msg__GimbalManagerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GimbalManagerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__STRUCT_H_
