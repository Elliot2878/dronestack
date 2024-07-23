// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/DebugValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_DEBUG'.
enum
{
  mavros_msgs__msg__DebugValue__TYPE_DEBUG = 0
};

/// Constant 'TYPE_DEBUG_VECT'.
enum
{
  mavros_msgs__msg__DebugValue__TYPE_DEBUG_VECT = 1
};

/// Constant 'TYPE_DEBUG_FLOAT_ARRAY'.
enum
{
  mavros_msgs__msg__DebugValue__TYPE_DEBUG_FLOAT_ARRAY = 2
};

/// Constant 'TYPE_NAMED_VALUE_FLOAT'.
enum
{
  mavros_msgs__msg__DebugValue__TYPE_NAMED_VALUE_FLOAT = 3
};

/// Constant 'TYPE_NAMED_VALUE_INT'.
enum
{
  mavros_msgs__msg__DebugValue__TYPE_NAMED_VALUE_INT = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/DebugValue in the package mavros_msgs.
/**
  * Msg for Debug MAVLink API
  *
  * Supported types:
  * DEBUG   https://mavlink.io/en/messages/common.html#DEBUG
  * DEBUG_VECTOR   https://mavlink.io/en/messages/common.html#DEBUG_VECT
  * DEBUG_FLOAT_ARRAY      https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY
  * NAMED_VALUE_FLOAT  https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
  * NAMED_VALUE_INT  https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT
 */
typedef struct mavros_msgs__msg__DebugValue
{
  std_msgs__msg__Header header;
  /// index value of DEBUG value (-1 if not indexed)
  int32_t index;
  /// Unique ID used to discriminate between DEBUG_FLOAT_ARRAYS (-1 if not used)
  int32_t array_id;
  /// value name/key
  rosidl_runtime_c__String name;
  /// float value for NAMED_VALUE_FLOAT and DEBUG
  float value_float;
  /// int value for NAMED_VALUE_INT
  int32_t value_int;
  /// DEBUG vector or array
  rosidl_runtime_c__float__Sequence data;
  uint8_t type;
} mavros_msgs__msg__DebugValue;

// Struct for a sequence of mavros_msgs__msg__DebugValue.
typedef struct mavros_msgs__msg__DebugValue__Sequence
{
  mavros_msgs__msg__DebugValue * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__DebugValue__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__STRUCT_H_
