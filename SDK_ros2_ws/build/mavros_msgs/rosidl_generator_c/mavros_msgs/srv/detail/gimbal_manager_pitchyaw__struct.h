// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/GimbalManagerPitchyaw.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__STRUCT_H_

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
  mavros_msgs__srv__GimbalManagerPitchyaw_Request__GIMBAL_MANAGER_FLAGS_RETRACT = 1ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_NEUTRAL'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
 */
enum
{
  mavros_msgs__srv__GimbalManagerPitchyaw_Request__GIMBAL_MANAGER_FLAGS_NEUTRAL = 2ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_ROLL_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
 */
enum
{
  mavros_msgs__srv__GimbalManagerPitchyaw_Request__GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_PITCH_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
 */
enum
{
  mavros_msgs__srv__GimbalManagerPitchyaw_Request__GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8ul
};

/// Constant 'GIMBAL_MANAGER_FLAGS_YAW_LOCK'.
/**
  * Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
 */
enum
{
  mavros_msgs__srv__GimbalManagerPitchyaw_Request__GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16ul
};

/// Struct defined in srv/GimbalManagerPitchyaw in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerPitchyaw_Request
{
  /// Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode). (-180 to 180 deg)
  float pitch;
  /// Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode). (-180 to 180 deg)
  float yaw;
  /// Pitch rate (positive to pitch up). (deg/s)
  float pitch_rate;
  /// Yaw rate (positive to yaw to the right). (deg/s)
  float yaw_rate;
  /// High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
  uint32_t flags;
  /// Component ID of gimbal device to address
  /// (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
  /// components. Send command multiple times for more than
  /// one gimbal (but not all gimbals).  Default Mavlink gimbal
  /// device ids: 154, 171-175
  uint8_t gimbal_device_id;
} mavros_msgs__srv__GimbalManagerPitchyaw_Request;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerPitchyaw_Request.
typedef struct mavros_msgs__srv__GimbalManagerPitchyaw_Request__Sequence
{
  mavros_msgs__srv__GimbalManagerPitchyaw_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerPitchyaw_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GimbalManagerPitchyaw in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerPitchyaw_Response
{
  bool success;
  /// raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__GimbalManagerPitchyaw_Response;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerPitchyaw_Response.
typedef struct mavros_msgs__srv__GimbalManagerPitchyaw_Response__Sequence
{
  mavros_msgs__srv__GimbalManagerPitchyaw_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerPitchyaw_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__STRUCT_H_
