// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/CommandAck.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_ACK__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_ACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CommandAck in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandAck_Request
{
  uint16_t command;
  uint8_t result;
  uint8_t progress;
  uint32_t result_param2;
} mavros_msgs__srv__CommandAck_Request;

// Struct for a sequence of mavros_msgs__srv__CommandAck_Request.
typedef struct mavros_msgs__srv__CommandAck_Request__Sequence
{
  mavros_msgs__srv__CommandAck_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandAck_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CommandAck in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandAck_Response
{
  bool success;
  /// raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__CommandAck_Response;

// Struct for a sequence of mavros_msgs__srv__CommandAck_Response.
typedef struct mavros_msgs__srv__CommandAck_Response__Sequence
{
  mavros_msgs__srv__CommandAck_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandAck_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_ACK__STRUCT_H_
