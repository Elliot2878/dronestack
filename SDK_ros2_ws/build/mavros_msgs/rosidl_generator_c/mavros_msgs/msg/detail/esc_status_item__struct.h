// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ESCStatusItem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_STATUS_ITEM__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ESC_STATUS_ITEM__STRUCT_H_

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

/// Struct defined in msg/ESCStatusItem in the package mavros_msgs.
/**
  * ESCStatusItem.msg
  *
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#ESC_STATUS
 */
typedef struct mavros_msgs__msg__ESCStatusItem
{
  std_msgs__msg__Header header;
  int32_t rpm;
  float voltage;
  float current;
} mavros_msgs__msg__ESCStatusItem;

// Struct for a sequence of mavros_msgs__msg__ESCStatusItem.
typedef struct mavros_msgs__msg__ESCStatusItem__Sequence
{
  mavros_msgs__msg__ESCStatusItem * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ESCStatusItem__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_STATUS_ITEM__STRUCT_H_
