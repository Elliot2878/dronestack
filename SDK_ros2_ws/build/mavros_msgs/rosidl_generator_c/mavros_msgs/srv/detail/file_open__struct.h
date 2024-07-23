// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/FileOpen.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_OPEN__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__FILE_OPEN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_READ'.
/**
  * open for read
 */
enum
{
  mavros_msgs__srv__FileOpen_Request__MODE_READ = 0
};

/// Constant 'MODE_WRITE'.
/**
  * open for write
 */
enum
{
  mavros_msgs__srv__FileOpen_Request__MODE_WRITE = 1
};

/// Constant 'MODE_CREATE'.
/**
  * do creat()
 */
enum
{
  mavros_msgs__srv__FileOpen_Request__MODE_CREATE = 2
};

// Include directives for member types
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FileOpen in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileOpen_Request
{
  rosidl_runtime_c__String file_path;
  uint8_t mode;
} mavros_msgs__srv__FileOpen_Request;

// Struct for a sequence of mavros_msgs__srv__FileOpen_Request.
typedef struct mavros_msgs__srv__FileOpen_Request__Sequence
{
  mavros_msgs__srv__FileOpen_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileOpen_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FileOpen in the package mavros_msgs.
typedef struct mavros_msgs__srv__FileOpen_Response
{
  uint32_t size;
  bool success;
  int32_t r_errno;
} mavros_msgs__srv__FileOpen_Response;

// Struct for a sequence of mavros_msgs__srv__FileOpen_Response.
typedef struct mavros_msgs__srv__FileOpen_Response__Sequence
{
  mavros_msgs__srv__FileOpen_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__FileOpen_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_OPEN__STRUCT_H_
