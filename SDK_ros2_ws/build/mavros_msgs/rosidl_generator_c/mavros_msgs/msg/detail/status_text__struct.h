// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/StatusText.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATUS_TEXT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__STATUS_TEXT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'EMERGENCY'.
/**
  * Severity levels
 */
enum
{
  mavros_msgs__msg__StatusText__EMERGENCY = 0
};

/// Constant 'ALERT'.
enum
{
  mavros_msgs__msg__StatusText__ALERT = 1
};

/// Constant 'CRITICAL'.
enum
{
  mavros_msgs__msg__StatusText__CRITICAL = 2
};

/// Constant 'ERROR'.
enum
{
  mavros_msgs__msg__StatusText__ERROR = 3
};

/// Constant 'WARNING'.
enum
{
  mavros_msgs__msg__StatusText__WARNING = 4
};

/// Constant 'NOTICE'.
enum
{
  mavros_msgs__msg__StatusText__NOTICE = 5
};

/// Constant 'INFO'.
enum
{
  mavros_msgs__msg__StatusText__INFO = 6
};

/// Constant 'DEBUG'.
enum
{
  mavros_msgs__msg__StatusText__DEBUG = 7
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'text'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StatusText in the package mavros_msgs.
/**
  * STATUSTEXT message representation
  * https://mavlink.io/en/messages/common.html#STATUSTEXT
 */
typedef struct mavros_msgs__msg__StatusText
{
  /// Fields
  std_msgs__msg__Header header;
  uint8_t severity;
  rosidl_runtime_c__String text;
} mavros_msgs__msg__StatusText;

// Struct for a sequence of mavros_msgs__msg__StatusText.
typedef struct mavros_msgs__msg__StatusText__Sequence
{
  mavros_msgs__msg__StatusText * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__StatusText__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__STATUS_TEXT__STRUCT_H_
