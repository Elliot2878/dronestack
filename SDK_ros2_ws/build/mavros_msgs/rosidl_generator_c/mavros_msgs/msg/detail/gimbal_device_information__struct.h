// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GimbalDeviceInformation.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_INFORMATION__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_INFORMATION__STRUCT_H_

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
  * GIMBAL_DEVICE_CAP_FLAGS
  *  Gimbal device supports a retracted position
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_RETRACT = 1ul
};

/// Constant 'CAP_FLAGS_HAS_NEUTRAL'.
/**
  * Gimbal device supports a horizontal, forward looking position, stabilized
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_NEUTRAL = 2ul
};

/// Constant 'CAP_FLAGS_HAS_ROLL_AXIS'.
/**
  * Gimbal device supports rotating around roll axis.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_ROLL_AXIS = 4ul
};

/// Constant 'CAP_FLAGS_HAS_ROLL_FOLLOW'.
/**
  * Gimbal device supports to follow a roll angle relative to the vehicle
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_ROLL_FOLLOW = 8ul
};

/// Constant 'CAP_FLAGS_HAS_ROLL_LOCK'.
/**
  * Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_ROLL_LOCK = 16ul
};

/// Constant 'CAP_FLAGS_HAS_PITCH_AXIS'.
/**
  * Gimbal device supports rotating around pitch axis.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_PITCH_AXIS = 32ul
};

/// Constant 'CAP_FLAGS_HAS_PITCH_FOLLOW'.
/**
  * Gimbal device supports to follow a pitch angle relative to the vehicle
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_PITCH_FOLLOW = 64ul
};

/// Constant 'CAP_FLAGS_HAS_PITCH_LOCK'.
/**
  * Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_PITCH_LOCK = 128ul
};

/// Constant 'CAP_FLAGS_HAS_YAW_AXIS'.
/**
  * Gimbal device supports rotating around yaw axis.
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_YAW_AXIS = 256ul
};

/// Constant 'CAP_FLAGS_HAS_YAW_FOLLOW'.
/**
  * Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_YAW_FOLLOW = 512ul
};

/// Constant 'CAP_FLAGS_HAS_YAW_LOCK'.
/**
  * Gimbal device supports locking to an absolute heading (often this is an option available)
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_HAS_YAW_LOCK = 1024ul
};

/// Constant 'CAP_FLAGS_SUPPORTS_INFINITE_YAW'.
/**
  * Gimbal device supports yawing/panning infinetely (e.g. using slip disk).
 */
enum
{
  mavros_msgs__msg__GimbalDeviceInformation__CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'vendor_name'
// Member 'model_name'
// Member 'custom_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GimbalDeviceInformation in the package mavros_msgs.
/**
  * MAVLink message: GIMBAL_DEVICE_INFORMATION
  * https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION
 */
typedef struct mavros_msgs__msg__GimbalDeviceInformation
{
  std_msgs__msg__Header header;
  /// Name of the gimbal vendor.
  rosidl_runtime_c__String vendor_name;
  /// Name of the gimbal model.
  rosidl_runtime_c__String model_name;
  /// Custom name of the gimbal given to it by the user.
  rosidl_runtime_c__String custom_name;
  /// Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
  uint32_t firmware_version;
  /// Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
  uint32_t hardware_version;
  /// UID of gimbal hardware (0 if unknown).
  uint64_t uid;
  /// Bitmap of gimbal capability flags - see GIMBAL_DEVICE_CAP_FLAGS
  uint32_t cap_flags;
  /// Bitmap for use for gimbal-specific capability flags.
  uint16_t custom_cap_flags;
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
} mavros_msgs__msg__GimbalDeviceInformation;

// Struct for a sequence of mavros_msgs__msg__GimbalDeviceInformation.
typedef struct mavros_msgs__msg__GimbalDeviceInformation__Sequence
{
  mavros_msgs__msg__GimbalDeviceInformation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GimbalDeviceInformation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_INFORMATION__STRUCT_H_
