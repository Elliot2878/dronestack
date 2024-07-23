// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/MessageInterval.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MessageInterval in the package mavros_msgs.
typedef struct mavros_msgs__srv__MessageInterval_Request
{
  uint32_t message_id;
  float message_rate;
} mavros_msgs__srv__MessageInterval_Request;

// Struct for a sequence of mavros_msgs__srv__MessageInterval_Request.
typedef struct mavros_msgs__srv__MessageInterval_Request__Sequence
{
  mavros_msgs__srv__MessageInterval_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__MessageInterval_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MessageInterval in the package mavros_msgs.
typedef struct mavros_msgs__srv__MessageInterval_Response
{
  bool success;
} mavros_msgs__srv__MessageInterval_Response;

// Struct for a sequence of mavros_msgs__srv__MessageInterval_Response.
typedef struct mavros_msgs__srv__MessageInterval_Response__Sequence
{
  mavros_msgs__srv__MessageInterval_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__MessageInterval_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__STRUCT_H_
