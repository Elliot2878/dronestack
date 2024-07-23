// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/CommandVtolTransition.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_VTOL_TRANSITION__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_VTOL_TRANSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_MC'.
/**
  * MAV_VTOL_STATE
 */
enum
{
  mavros_msgs__srv__CommandVtolTransition_Request__STATE_MC = 3
};

/// Constant 'STATE_FW'.
enum
{
  mavros_msgs__srv__CommandVtolTransition_Request__STATE_FW = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in srv/CommandVtolTransition in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandVtolTransition_Request
{
  /// MAVLink command: DO_VTOL_TRANSITION
  /// https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION
  std_msgs__msg__Header header;
  /// See enum MAV_VTOL_STATE.
  uint8_t state;
} mavros_msgs__srv__CommandVtolTransition_Request;

// Struct for a sequence of mavros_msgs__srv__CommandVtolTransition_Request.
typedef struct mavros_msgs__srv__CommandVtolTransition_Request__Sequence
{
  mavros_msgs__srv__CommandVtolTransition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandVtolTransition_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CommandVtolTransition in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandVtolTransition_Response
{
  bool success;
  /// Raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__CommandVtolTransition_Response;

// Struct for a sequence of mavros_msgs__srv__CommandVtolTransition_Response.
typedef struct mavros_msgs__srv__CommandVtolTransition_Response__Sequence
{
  mavros_msgs__srv__CommandVtolTransition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandVtolTransition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_VTOL_TRANSITION__STRUCT_H_
