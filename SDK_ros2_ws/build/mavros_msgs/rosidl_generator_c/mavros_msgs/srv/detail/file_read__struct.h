// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/FileRead.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_READ__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__FILE_READ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FileRead in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileRead_Request
{
  rosidl_runtime_c__String file_path;
  uint64_t offset;
  uint64_t size;
} mavros_msgs__srv__FileRead_Request;

// Struct for a sequence of mavros_msgs__srv__FileRead_Request.
typedef struct mavros_msgs__srv__FileRead_Request__Sequence
{
  mavros_msgs__srv__FileRead_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileRead_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/FileRead in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileRead_Response
{
  rosidl_runtime_c__uint8__Sequence data;
  bool success;
  int32_t r_errno;
} mavros_msgs__srv__FileRead_Response;

// Struct for a sequence of mavros_msgs__srv__FileRead_Response.
typedef struct mavros_msgs__srv__FileRead_Response__Sequence
{
  mavros_msgs__srv__FileRead_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileRead_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_READ__STRUCT_H_
