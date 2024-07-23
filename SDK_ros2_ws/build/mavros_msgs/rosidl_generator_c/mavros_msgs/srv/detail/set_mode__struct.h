// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/SetMode.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__SET_MODE__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAV_MODE_PREFLIGHT'.
/**
  * basic modes from MAV_MODE
 */
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_PREFLIGHT = 0
};

/// Constant 'MAV_MODE_STABILIZE_DISARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_STABILIZE_DISARMED = 80
};

/// Constant 'MAV_MODE_STABILIZE_ARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_STABILIZE_ARMED = 208
};

/// Constant 'MAV_MODE_MANUAL_DISARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_MANUAL_DISARMED = 64
};

/// Constant 'MAV_MODE_MANUAL_ARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_MANUAL_ARMED = 192
};

/// Constant 'MAV_MODE_GUIDED_DISARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_GUIDED_DISARMED = 88
};

/// Constant 'MAV_MODE_GUIDED_ARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_GUIDED_ARMED = 216
};

/// Constant 'MAV_MODE_AUTO_DISARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_AUTO_DISARMED = 92
};

/// Constant 'MAV_MODE_AUTO_ARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_AUTO_ARMED = 220
};

/// Constant 'MAV_MODE_TEST_DISARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_TEST_DISARMED = 66
};

/// Constant 'MAV_MODE_TEST_ARMED'.
enum
{
  mavros_msgs__srv__SetMode_Request__MAV_MODE_TEST_ARMED = 194
};

// Include directives for member types
// Member 'custom_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMode in the package mavros_msgs.
typedef struct mavros_msgs__srv__SetMode_Request
{
  /// filled by MAV_MODE enum value or 0 if custom_mode != ''
  uint8_t base_mode;
  /// string mode representation or integer
  rosidl_runtime_c__String custom_mode;
} mavros_msgs__srv__SetMode_Request;

// Struct for a sequence of mavros_msgs__srv__SetMode_Request.
typedef struct mavros_msgs__srv__SetMode_Request__Sequence
{
  mavros_msgs__srv__SetMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__SetMode_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetMode in the package mavros_msgs.
typedef struct mavros_msgs__srv__SetMode_Response
{
  /// Mode known/parsed correctly and SET_MODE are sent
  bool mode_sent;
} mavros_msgs__srv__SetMode_Response;

// Struct for a sequence of mavros_msgs__srv__SetMode_Response.
typedef struct mavros_msgs__srv__SetMode_Response__Sequence
{
  mavros_msgs__srv__SetMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__SetMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__SET_MODE__STRUCT_H_
