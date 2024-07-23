// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/ParamGet.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_GET__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_GET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'param_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ParamGet in the package mavros_msgs.
typedef struct mavros_msgs__srv__ParamGet_Request
{
  /// XXX DEPRECATED: use ros2 parameters api instead
  rosidl_runtime_c__String param_id;
} mavros_msgs__srv__ParamGet_Request;

// Struct for a sequence of mavros_msgs__srv__ParamGet_Request.
typedef struct mavros_msgs__srv__ParamGet_Request__Sequence
{
  mavros_msgs__srv__ParamGet_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__ParamGet_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'value'
#include "mavros_msgs/msg/detail/param_value__struct.h"

/// Struct defined in srv/ParamGet in the package mavros_msgs.
typedef struct mavros_msgs__srv__ParamGet_Response
{
  bool success;
  mavros_msgs__msg__ParamValue value;
} mavros_msgs__srv__ParamGet_Response;

// Struct for a sequence of mavros_msgs__srv__ParamGet_Response.
typedef struct mavros_msgs__srv__ParamGet_Response__Sequence
{
  mavros_msgs__srv__ParamGet_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__ParamGet_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_GET__STRUCT_H_
