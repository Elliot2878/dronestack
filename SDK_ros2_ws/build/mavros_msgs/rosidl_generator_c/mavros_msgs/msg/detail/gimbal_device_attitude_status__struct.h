// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GimbalDeviceAttitudeStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FLAGS_RETRACT'.
/**
  * GIMBAL_DEVICE_FLAGS
  *  Set to retracted safe position (no stabilization), takes presedence over all other flags.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__FLAGS_RETRACT = 1
};

/// Constant 'FLAGS_NEUTRAL'.
/**
  * Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__FLAGS_NEUTRAL = 2
};

/// Constant 'FLAGS_ROLL_LOCK'.
/**
  * Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__FLAGS_ROLL_LOCK = 4
};

/// Constant 'FLAGS_PITCH_LOCK'.
/**
  * Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__FLAGS_PITCH_LOCK = 8
};

/// Constant 'FLAGS_YAW_LOCK'.
/**
  * Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__FLAGS_YAW_LOCK = 16
};

/// Constant 'ERROR_FLAGS_AT_ROLL_LIMIT'.
/**
  * GIMBAL_DEVICE_ERROR_FLAGS
  *  Gimbal device is limited by hardware roll limit.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_AT_ROLL_LIMIT = 1ul
};

/// Constant 'ERROR_FLAGS_AT_PITCH_LIMIT'.
/**
  * Gimbal device is limited by hardware pitch limit.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_AT_PITCH_LIMIT = 2ul
};

/// Constant 'ERROR_FLAGS_AT_YAW_LIMIT'.
/**
  * Gimbal device is limited by hardware yaw limit.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_AT_YAW_LIMIT = 4ul
};

/// Constant 'ERROR_FLAGS_ENCODER_ERROR'.
/**
  * There is an error with the gimbal encoders.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_ENCODER_ERROR = 8ul
};

/// Constant 'ERROR_FLAGS_POWER_ERROR'.
/**
  * There is an error with the gimbal power source.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_POWER_ERROR = 16ul
};

/// Constant 'ERROR_FLAGS_MOTOR_ERROR'.
/**
  * There is an error with the gimbal motor's.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_MOTOR_ERROR = 32ul
};

/// Constant 'ERROR_FLAGS_SOFTWARE_ERROR'.
/**
  * There is an error with the gimbal's software.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_SOFTWARE_ERROR = 64ul
};

/// Constant 'ERROR_FLAGS_COMMS_ERROR'.
/**
  * There is an error with the gimbal's communication.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_COMMS_ERROR = 128ul
};

/// Constant 'ERROR_FLAGS_CALIBRATION_RUNNING'.
/**
  * Gimbal is currently calibrating.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus__ERROR_FLAGS_CALIBRATION_RUNNING = 256ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'q'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/GimbalDeviceAttitudeStatus in the package mavros_msgs.
/**
  * MAVLink message: GIMBAL_DEVICE_ATTITUDE_STATUS
  * https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS
 */
typedef struct mavros_msgs__msg__GimbalDeviceAttitudeStatus
{
  std_msgs__msg__Header header;
  /// System ID
  uint8_t target_system;
  /// Component ID
  uint8_t target_component;
  /// Current gimbal flags set (bitwise) - See GIMBAL_DEVICE_FLAGS
  uint16_t flags;
  /// Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
  geometry_msgs__msg__Quaternion q;
  /// X component of angular velocity (NaN if unknown)
  float angular_velocity_x;
  /// Y component of angular velocity (NaN if unknown)
  float angular_velocity_y;
  /// Z component of angular velocity (NaN if unknown)
  float angular_velocity_z;
  /// Failure flags (0 for no failure) (bitwise) - See GIMBAL_DEVICE_ERROR_FLAGS
  uint32_t failure_flags;
} mavros_msgs__msg__GimbalDeviceAttitudeStatus;

// Struct for a sequence of mavros_msgs__msg__GimbalDeviceAttitudeStatus.
typedef struct mavros_msgs__msg__GimbalDeviceAttitudeStatus__Sequence
{
  mavros_msgs__msg__GimbalDeviceAttitudeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GimbalDeviceAttitudeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__STRUCT_H_
