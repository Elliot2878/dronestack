// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/ParamSetV2.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_SET_V2__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_SET_V2__STRUCT_H_

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
// Member 'value'
#include "rcl_interfaces/msg/detail/parameter_value__struct.h"

/// Struct defined in srv/ParamSetV2 in the package mavros_msgs.
typedef struct mavros_msgs__srv__ParamSetV2_Request
{
  bool force_set;
  rosidl_runtime_c__String param_id;
  rcl_interfaces__msg__ParameterValue value;
} mavros_msgs__srv__ParamSetV2_Request;

// Struct for a sequence of mavros_msgs__srv__ParamSetV2_Request.
typedef struct mavros_msgs__srv__ParamSetV2_Request__Sequence
{
  mavros_msgs__srv__ParamSetV2_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__ParamSetV2_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'value'
// already included above
// #include "rcl_interfaces/msg/detail/parameter_value__struct.h"

/// Struct defined in srv/ParamSetV2 in the package mavros_msgs.
typedef struct mavros_msgs__srv__ParamSetV2_Response
{
  bool success;
  rcl_interfaces__msg__ParameterValue value;
} mavros_msgs__srv__ParamSetV2_Response;

// Struct for a sequence of mavros_msgs__srv__ParamSetV2_Response.
typedef struct mavros_msgs__srv__ParamSetV2_Response__Sequence
{
  mavros_msgs__srv__ParamSetV2_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__ParamSetV2_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_SET_V2__STRUCT_H_
