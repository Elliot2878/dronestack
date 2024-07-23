// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GimbalDeviceSetAttitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__STRUCT_H_

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
  *  Based on GIMBAL_DEVICE_FLAGS_RETRACT
 */
enum
{
  mavros_msgs__msg__GimbalDeviceSetAttitude__FLAGS_RETRACT = 1
};

/// Constant 'FLAGS_NEUTRAL'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
 */
enum
{
  mavros_msgs__msg__GimbalDeviceSetAttitude__FLAGS_NEUTRAL = 2
};

/// Constant 'FLAGS_ROLL_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalDeviceSetAttitude__FLAGS_ROLL_LOCK = 4
};

/// Constant 'FLAGS_PITCH_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalDeviceSetAttitude__FLAGS_PITCH_LOCK = 8
};

/// Constant 'FLAGS_YAW_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalDeviceSetAttitude__FLAGS_YAW_LOCK = 16
};

// Include directives for member types
// Member 'q'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/GimbalDeviceSetAttitude in the package mavros_msgs.
/**
  * MAVLink message: GIMBAL_DEVICE_SET_ATTITUDE
  * https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE
 */
typedef struct mavros_msgs__msg__GimbalDeviceSetAttitude
{
  /// System ID
  uint8_t target_system;
  /// Component ID
  uint8_t target_component;
  /// Low level gimbal flags (bitwise) - See GIMBAL_DEVICE_FLAGS
  uint16_t flags;
  /// Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
  geometry_msgs__msg__Quaternion q;
  /// X component of angular velocity, positive is rolling to the right, NaN to be ignored.
  float angular_velocity_x;
  /// Y component of angular velocity, positive is pitching up, NaN to be ignored.
  float angular_velocity_y;
  /// Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
  float angular_velocity_z;
} mavros_msgs__msg__GimbalDeviceSetAttitude;

// Struct for a sequence of mavros_msgs__msg__GimbalDeviceSetAttitude.
typedef struct mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence
{
  mavros_msgs__msg__GimbalDeviceSetAttitude * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GimbalDeviceSetAttitude__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__STRUCT_H_