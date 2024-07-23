// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/MountConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_RETRACT'.
/**
  * MAV_MOUNT_MODE
 */
enum
{
  mavros_msgs__srv__MountConfigure_Request__MODE_RETRACT = 0
};

/// Constant 'MODE_NEUTRAL'.
enum
{
  mavros_msgs__srv__MountConfigure_Request__MODE_NEUTRAL = 1
};

/// Constant 'MODE_MAVLINK_TARGETING'.
enum
{
  mavros_msgs__srv__MountConfigure_Request__MODE_MAVLINK_TARGETING = 2
};

/// Constant 'MODE_RC_TARGETING'.
enum
{
  mavros_msgs__srv__MountConfigure_Request__MODE_RC_TARGETING = 3
};

/// Constant 'MODE_GPS_POINT'.
enum
{
  mavros_msgs__srv__MountConfigure_Request__MODE_GPS_POINT = 4
};

/// Constant 'INPUT_ANGLE_BODY_FRAME'.
/**
  * MOUNT_INPUT
 */
enum
{
  mavros_msgs__srv__MountConfigure_Request__INPUT_ANGLE_BODY_FRAME = 0
};

/// Constant 'INPUT_ANGULAR_RATE'.
enum
{
  mavros_msgs__srv__MountConfigure_Request__INPUT_ANGULAR_RATE = 1
};

/// Constant 'INPUT_ANGLE_ABSOLUTE_FRAME'.
enum
{
  mavros_msgs__srv__MountConfigure_Request__INPUT_ANGLE_ABSOLUTE_FRAME = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in srv/MountConfigure in the package mavros_msgs.
typedef struct mavros_msgs__srv__MountConfigure_Request
{
  std_msgs__msg__Header header;
  /// See enum MAV_MOUNT_MODE.
  uint8_t mode;
  /// stabilize roll? (1 = yes, 0 = no)
  bool stabilize_roll;
  /// stabilize pitch? (1 = yes, 0 = no)
  bool stabilize_pitch;
  /// stabilize yaw? (1 = yes, 0 = no)
  bool stabilize_yaw;
  /// roll input (See enum MOUNT_INPUT)
  uint8_t roll_input;
  /// pitch input (See enum MOUNT_INPUT)
  uint8_t pitch_input;
  /// yaw input (See enum MOUNT_INPUT)
  uint8_t yaw_input;
} mavros_msgs__srv__MountConfigure_Request;

// Struct for a sequence of mavros_msgs__srv__MountConfigure_Request.
typedef struct mavros_msgs__srv__MountConfigure_Request__Sequence
{
  mavros_msgs__srv__MountConfigure_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__MountConfigure_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MountConfigure in the package mavros_msgs.
typedef struct mavros_msgs__srv__MountConfigure_Response
{
  bool success;
  /// raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__MountConfigure_Response;

// Struct for a sequence of mavros_msgs__srv__MountConfigure_Response.
typedef struct mavros_msgs__srv__MountConfigure_Response__Sequence
{
  mavros_msgs__srv__MountConfigure_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__MountConfigure_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__STRUCT_H_
