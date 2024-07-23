// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/GimbalManagerConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GimbalManagerConfigure in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerConfigure_Request
{
  /// Sysid for primary control (0: no one in control,
  /// -1: leave unchanged, -2: set itself in control
  /// (for missions where the own sysid is still unknown),
  /// -3: remove control if currently in control).
  int16_t sysid_primary;
  /// Compid for primary control (0: no one in control,
  /// -1: leave unchanged, -2: set itself in control
  /// (for missions where the own sysid is still unknown),
  /// -3: remove control if currently in control).
  int16_t compid_primary;
  /// Sysid for secondary control (0: no one in control,
  /// -1: leave unchanged, -2: set itself in control
  /// (for missions where the own sysid is still unknown),
  /// -3: remove control if currently in control).
  int16_t sysid_secondary;
  /// Compid for secondary control (0: no one in control,
  /// -1: leave unchanged, -2: set itself in control
  /// (for missions where the own sysid is still unknown),
  /// -3: remove control if currently in control).
  int16_t compid_secondary;
  /// Component ID of gimbal device to address
  /// (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
  /// components. Send command multiple times for more than
  /// one gimbal (but not all gimbals).
  /// Note: Default Mavlink gimbal device ids: 154, 171-175
  uint8_t gimbal_device_id;
} mavros_msgs__srv__GimbalManagerConfigure_Request;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerConfigure_Request.
typedef struct mavros_msgs__srv__GimbalManagerConfigure_Request__Sequence
{
  mavros_msgs__srv__GimbalManagerConfigure_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerConfigure_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GimbalManagerConfigure in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerConfigure_Response
{
  bool success;
  /// raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__GimbalManagerConfigure_Response;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerConfigure_Response.
typedef struct mavros_msgs__srv__GimbalManagerConfigure_Response__Sequence
{
  mavros_msgs__srv__GimbalManagerConfigure_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerConfigure_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__STRUCT_H_
