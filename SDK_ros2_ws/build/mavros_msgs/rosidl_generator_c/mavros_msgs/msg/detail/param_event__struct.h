// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ParamEvent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'param_id'
#include "rosidl_runtime_c/string.h"
// Member 'value'
#include "rcl_interfaces/msg/detail/parameter_value__struct.h"

/// Struct defined in msg/ParamEvent in the package mavros_msgs.
/**
  * Parameter Event
  *
  * That messages replaces mavros_msgs/Param from mavros v1.
  * Reason for that: ROS2 have native message for parameters
  *
  * ROS2 also have it's own ParameterEvent stream, which could be used
  * to get FCU updates too. But that message is simpler to use.
 */
typedef struct mavros_msgs__msg__ParamEvent
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String param_id;
  rcl_interfaces__msg__ParameterValue value;
  uint16_t param_index;
  uint16_t param_count;
} mavros_msgs__msg__ParamEvent;

// Struct for a sequence of mavros_msgs__msg__ParamEvent.
typedef struct mavros_msgs__msg__ParamEvent__Sequence
{
  mavros_msgs__msg__ParamEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ParamEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__STRUCT_H_
