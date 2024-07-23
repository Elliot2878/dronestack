// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ESCStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ESC_STATUS__STRUCT_H_

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
// Member 'esc_status'
#include "mavros_msgs/msg/detail/esc_status_item__struct.h"

/// Struct defined in msg/ESCStatus in the package mavros_msgs.
/**
  * ESCStatus.msg
  *
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#ESC_STATUS
 */
typedef struct mavros_msgs__msg__ESCStatus
{
  std_msgs__msg__Header header;
  mavros_msgs__msg__ESCStatusItem__Sequence esc_status;
} mavros_msgs__msg__ESCStatus;

// Struct for a sequence of mavros_msgs__msg__ESCStatus.
typedef struct mavros_msgs__msg__ESCStatus__Sequence
{
  mavros_msgs__msg__ESCStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ESCStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_STATUS__STRUCT_H_
