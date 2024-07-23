// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ESCInfoItem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__STRUCT_H_

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

/// Struct defined in msg/ESCInfoItem in the package mavros_msgs.
/**
  * ESCInfoItem.msg
  *
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#ESC_INFO
 */
typedef struct mavros_msgs__msg__ESCInfoItem
{
  std_msgs__msg__Header header;
  uint16_t failure_flags;
  uint32_t error_count;
  int32_t temperature;
} mavros_msgs__msg__ESCInfoItem;

// Struct for a sequence of mavros_msgs__msg__ESCInfoItem.
typedef struct mavros_msgs__msg__ESCInfoItem__Sequence
{
  mavros_msgs__msg__ESCInfoItem * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ESCInfoItem__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__STRUCT_H_
