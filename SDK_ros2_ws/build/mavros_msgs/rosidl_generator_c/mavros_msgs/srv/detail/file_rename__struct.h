// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/FileRename.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_RENAME__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__FILE_RENAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'old_path'
// Member 'new_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FileRename in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileRename_Request
{
  rosidl_runtime_c__String old_path;
  rosidl_runtime_c__String new_path;
} mavros_msgs__srv__FileRename_Request;

// Struct for a sequence of mavros_msgs__srv__FileRename_Request.
typedef struct mavros_msgs__srv__FileRename_Request__Sequence
{
  mavros_msgs__srv__FileRename_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileRename_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FileRename in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileRename_Response
{
  bool success;
  int32_t r_errno;
} mavros_msgs__srv__FileRename_Response;

// Struct for a sequence of mavros_msgs__srv__FileRename_Response.
typedef struct mavros_msgs__srv__FileRename_Response__Sequence
{
  mavros_msgs__srv__FileRename_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileRename_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_RENAME__STRUCT_H_
