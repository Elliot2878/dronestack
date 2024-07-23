// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/FileList.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_LIST__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__FILE_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dir_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FileList in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileList_Request
{
  rosidl_runtime_c__String dir_path;
} mavros_msgs__srv__FileList_Request;

// Struct for a sequence of mavros_msgs__srv__FileList_Request.
typedef struct mavros_msgs__srv__FileList_Request__Sequence
{
  mavros_msgs__srv__FileList_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileList_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'list'
#include "mavros_msgs/msg/detail/file_entry__struct.h"

/// Struct defined in srv/FileList in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileList_Response
{
  mavros_msgs__msg__FileEntry__Sequence list;
  bool success;
  int32_t r_errno;
} mavros_msgs__srv__FileList_Response;

// Struct for a sequence of mavros_msgs__srv__FileList_Response.
typedef struct mavros_msgs__srv__FileList_Response__Sequence
{
  mavros_msgs__srv__FileList_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileList_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_LIST__STRUCT_H_
