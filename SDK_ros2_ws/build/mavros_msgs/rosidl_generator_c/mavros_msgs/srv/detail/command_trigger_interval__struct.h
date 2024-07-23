// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/CommandTriggerInterval.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_INTERVAL__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_INTERVAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CommandTriggerInterval in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandTriggerInterval_Request
{
  /// Trigger cycle_time (interval between to triggers) - set to 0 to ignore command
  float cycle_time;
  /// Camera shutter integration_time - set to 0 to ignore command
  float integration_time;
} mavros_msgs__srv__CommandTriggerInterval_Request;

// Struct for a sequence of mavros_msgs__srv__CommandTriggerInterval_Request.
typedef struct mavros_msgs__srv__CommandTriggerInterval_Request__Sequence
{
  mavros_msgs__srv__CommandTriggerInterval_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandTriggerInterval_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CommandTriggerInterval in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandTriggerInterval_Response
{
  bool success;
  uint8_t result;
} mavros_msgs__srv__CommandTriggerInterval_Response;

// Struct for a sequence of mavros_msgs__srv__CommandTriggerInterval_Response.
typedef struct mavros_msgs__srv__CommandTriggerInterval_Response__Sequence
{
  mavros_msgs__srv__CommandTriggerInterval_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandTriggerInterval_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_INTERVAL__STRUCT_H_
