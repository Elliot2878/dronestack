// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ParamValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ParamValue in the package mavros_msgs.
/**
  * Parameter value storage type.
  *
  * Integer and float fields:
  *
  * if integer != 0: it is integer value
  * else if real != 0.0: it is float value
  * else: it is zero.
 */
typedef struct mavros_msgs__msg__ParamValue
{
  /// XXX DEPRECATED: replaced by rmw_interfaces/ParameterValue
  int64_t integer;
  double real;
} mavros_msgs__msg__ParamValue;

// Struct for a sequence of mavros_msgs__msg__ParamValue.
typedef struct mavros_msgs__msg__ParamValue__Sequence
{
  mavros_msgs__msg__ParamValue * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ParamValue__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__STRUCT_H_
