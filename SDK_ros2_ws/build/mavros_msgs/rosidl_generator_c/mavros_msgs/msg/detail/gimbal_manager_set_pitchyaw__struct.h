// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GimbalManagerSetPitchyaw.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_SET_PITCHYAW__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_SET_PITCHYAW__STRUCT_H_

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
  mavros_msgs__msg__GimbalManagerSetPitchyaw__GIMBAL_MANAGER_FLAGS_RETRACT = 1ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_NEUTRAL'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
 */
enum
{
  mavros_msgs__msg__GimbalManagerSetPitchyaw__GIMBAL_MANAGER_FLAGS_NEUTRAL = 2ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_ROLL_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalManagerSetPitchyaw__GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_PITCH_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalManagerSetPitchyaw__GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_YAW_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
 */
enum
{
  mavros_msgs__msg__GimbalManagerSetPitchyaw__GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16ul
};

/// Struct defined in msg/GimbalManagerSetPitchyaw in the package mavros_msgs.
/**
  * MAVLink message: GIMBAL_MANAGER_SET_PITCHYAW
  * https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW
  * Note that this message structure is identical also to GIMBAL_MANAGER_SET_MANUAL_CONTROL and is 
  * reused as such by the plugin
  * https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL
 */
typedef struct mavros_msgs__msg__GimbalManagerSetPitchyaw
{
  /// System ID
  uint8_t target_system;
  /// Component ID
  uint8_t target_component;
  /// High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
  uint32_t flags;
  /// Component ID of gimbal device to address
  /// (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
  /// components. Send command multiple times for more than
  /// one gimbal (but not all gimbals).  Default Mavlink gimbal
  /// device ids: 154, 171-175
  uint8_t gimbal_device_id;
  /// Pitch angle (positive: up, negative: down, NaN to be ignored).
  float pitch;
  /// Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
  float yaw;
  /// Pitch angular rate (positive: up, negative: down, NaN to be ignored).
  float pitch_rate;
  /// Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
  float yaw_rate;
} mavros_msgs__msg__GimbalManagerSetPitchyaw;

// Struct for a sequence of mavros_msgs__msg__GimbalManagerSetPitchyaw.
typedef struct mavros_msgs__msg__GimbalManagerSetPitchyaw__Sequence
{
  mavros_msgs__msg__GimbalManagerSetPitchyaw * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GimbalManagerSetPitchyaw__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_SET_PITCHYAW__STRUCT_H_
