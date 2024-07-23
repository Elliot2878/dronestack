// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/StatusEvent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__STRUCT_H_

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
  mavros_msgs__msg__StatusEvent__EMERGENCY = 0
};

/// Constant 'ALERT'.
enum
{
  mavros_msgs__msg__StatusEvent__ALERT = 1
};

/// Constant 'CRITICAL'.
enum
{
  mavros_msgs__msg__StatusEvent__CRITICAL = 2
};

/// Constant 'ERROR'.
enum
{
  mavros_msgs__msg__StatusEvent__ERROR = 3
};

/// Constant 'WARNING'.
enum
{
  mavros_msgs__msg__StatusEvent__WARNING = 4
};

/// Constant 'NOTICE'.
enum
{
  mavros_msgs__msg__StatusEvent__NOTICE = 5
};

/// Constant 'INFO'.
enum
{
  mavros_msgs__msg__StatusEvent__INFO = 6
};

/// Constant 'DEBUG'.
enum
{
  mavros_msgs__msg__StatusEvent__DEBUG = 7
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/StatusEvent in the package mavros_msgs.
/**
  * EVENT message representation
  * https://mavlink.io/en/messages/common.html#EVENT
 */
typedef struct mavros_msgs__msg__StatusEvent
{
  /// Fields
  std_msgs__msg__Header header;
  uint8_t severity;
  uint32_t px4_id;
  uint8_t arguments[40];
  uint16_t sequence;
} mavros_msgs__msg__StatusEvent;

// Struct for a sequence of mavros_msgs__msg__StatusEvent.
typedef struct mavros_msgs__msg__StatusEvent__Sequence
{
  mavros_msgs__msg__StatusEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__StatusEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__STRUCT_H_
