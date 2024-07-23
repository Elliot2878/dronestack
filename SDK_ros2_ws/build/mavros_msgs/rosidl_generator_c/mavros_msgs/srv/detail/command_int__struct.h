// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/CommandInt.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_INT__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_INT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CommandInt in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandInt_Request
{
  /// send this command in broadcast mode
  bool broadcast;
  uint8_t frame;
  uint16_t command;
  uint8_t current;
  uint8_t autocontinue;
  float param1;
  float param2;
  float param3;
  float param4;
  /// latitude in deg * 1E7 or local x * 1E4 m
  int32_t x;
  /// longitude in deg * 1E7 or local y * 1E4 m
  int32_t y;
  /// altitude
  float z;
} mavros_msgs__srv__CommandInt_Request;

// Struct for a sequence of mavros_msgs__srv__CommandInt_Request.
typedef struct mavros_msgs__srv__CommandInt_Request__Sequence
{
  mavros_msgs__srv__CommandInt_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandInt_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CommandInt in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandInt_Response
{
  bool success;
} mavros_msgs__srv__CommandInt_Response;

// Struct for a sequence of mavros_msgs__srv__CommandInt_Response.
typedef struct mavros_msgs__srv__CommandInt_Response__Sequence
{
  mavros_msgs__srv__CommandInt_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandInt_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_INT__STRUCT_H_
