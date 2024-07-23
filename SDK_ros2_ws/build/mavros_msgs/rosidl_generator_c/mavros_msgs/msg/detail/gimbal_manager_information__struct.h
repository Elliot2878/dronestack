// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GimbalManagerInformation.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_INFORMATION__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_INFORMATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CAP_FLAGS_HAS_RETRACT'.
/**
  * GIMBAL_MANAGER_CAP_FLAGS
  *  Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_RETRACT = 1ul
};

/// Constant 'CAP_FLAGS_HAS_NEUTRAL'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_NEUTRAL = 2ul
};

/// Constant 'CAP_FLAGS_HAS_ROLL_AXIS'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_ROLL_AXIS = 4ul
};

/// Constant 'CAP_FLAGS_HAS_ROLL_FOLLOW'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_ROLL_FOLLOW = 8ul
};

/// Constant 'CAP_FLAGS_HAS_ROLL_LOCK'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_ROLL_LOCK = 16ul
};

/// Constant 'CAP_FLAGS_HAS_PITCH_AXIS'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_PITCH_AXIS = 32ul
};

/// Constant 'CAP_FLAGS_HAS_PITCH_FOLLOW'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_PITCH_FOLLOW = 64ul
};

/// Constant 'CAP_FLAGS_HAS_PITCH_LOCK'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_PITCH_LOCK = 128ul
};

/// Constant 'CAP_FLAGS_HAS_YAW_AXIS'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_YAW_AXIS = 256ul
};

/// Constant 'CAP_FLAGS_HAS_YAW_FOLLOW'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_YAW_FOLLOW = 512ul
};

/// Constant 'CAP_FLAGS_HAS_YAW_LOCK'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_HAS_YAW_LOCK = 1024ul
};

/// Constant 'CAP_FLAGS_SUPPORTS_INFINITE_YAW'.
/**
  * Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048ul
};

/// Constant 'CAP_FLAGS_CAN_POINT_LOCATION_LOCAL'.
/**
  * Gimbal manager supports to point to a local position.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536ul
};

/// Constant 'CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL'.
/**
  * Gimbal manager supports to point to a global latitude, longitude, altitude position.
 */
enum
{
  mavros_msgs__msg__GimbalManagerInformation__CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GimbalManagerInformation in the package mavros_msgs.
/**
  * MAVLink message: GIMBAL_MANAGER_INFORMATION
  * https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION
 */
typedef struct mavros_msgs__msg__GimbalManagerInformation
{
  std_msgs__msg__Header header;
  /// Bitmap of gimbal capability flags - see GIMBAL_MANAGER_CAP_FLAGS
  uint32_t cap_flags;
  /// Gimbal device ID that this gimbal manager is responsible for.
  uint8_t gimbal_device_id;
  /// Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
  float roll_min;
  /// Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
  float roll_max;
  /// Minimum pitch angle (positive: up, negative: down)
  float pitch_min;
  /// Maximum pitch angle (positive: up, negative: down)
  float pitch_max;
  /// Minimum yaw angle (positive: to the right, negative: to the left)
  float yaw_min;
  /// Maximum yaw angle (positive: to the right, negative: to the left)
  float yaw_max;
} mavros_msgs__msg__GimbalManagerInformation;

// Struct for a sequence of mavros_msgs__msg__GimbalManagerInformation.
typedef struct mavros_msgs__msg__GimbalManagerInformation__Sequence
{
  mavros_msgs__msg__GimbalManagerInformation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GimbalManagerInformation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_INFORMATION__STRUCT_H_
