// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/FileChecksum.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_CHECKSUM__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__FILE_CHECKSUM__STRUCT_H_

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

/// Struct defined in srv/FileChecksum in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileChecksum_Request
{
  rosidl_runtime_c__String file_path;
} mavros_msgs__srv__FileChecksum_Request;

// Struct for a sequence of mavros_msgs__srv__FileChecksum_Request.
typedef struct mavros_msgs__srv__FileChecksum_Request__Sequence
{
  mavros_msgs__srv__FileChecksum_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileChecksum_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FileChecksum in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileChecksum_Response
{
  uint32_t crc32;
  bool success;
  int32_t r_errno;
} mavros_msgs__srv__FileChecksum_Response;

// Struct for a sequence of mavros_msgs__srv__FileChecksum_Response.
typedef struct mavros_msgs__srv__FileChecksum_Response__Sequence
{
  mavros_msgs__srv__FileChecksum_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileChecksum_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_CHECKSUM__STRUCT_H_
