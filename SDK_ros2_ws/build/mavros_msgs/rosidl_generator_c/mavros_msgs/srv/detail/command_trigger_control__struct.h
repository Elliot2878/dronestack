// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/CommandTriggerControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CommandTriggerControl in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandTriggerControl_Request
{
  /// Trigger enable/disable
  bool trigger_enable;
  /// Reset the trigger sequence
  bool sequence_reset;
  /// Pause triggering, but without switching the camera off or retracting it.
  bool trigger_pause;
} mavros_msgs__srv__CommandTriggerControl_Request;

// Struct for a sequence of mavros_msgs__srv__CommandTriggerControl_Request.
typedef struct mavros_msgs__srv__CommandTriggerControl_Request__Sequence
{
  mavros_msgs__srv__CommandTriggerControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandTriggerControl_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CommandTriggerControl in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandTriggerControl_Response
{
  bool success;
  uint8_t result;
} mavros_msgs__srv__CommandTriggerControl_Response;

// Struct for a sequence of mavros_msgs__srv__CommandTriggerControl_Response.
typedef struct mavros_msgs__srv__CommandTriggerControl_Response__Sequence
{
  mavros_msgs__srv__CommandTriggerControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandTriggerControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__STRUCT_H_
