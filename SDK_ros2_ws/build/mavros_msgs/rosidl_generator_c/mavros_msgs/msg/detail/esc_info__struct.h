// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/ESCInfo.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_INFO__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ESC_INFO__STRUCT_H_

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
// Member 'esc_info'
#include "mavros_msgs/msg/detail/esc_info_item__struct.h"

/// Struct defined in msg/ESCInfo in the package mavros_msgs.
/**
  * ESCInfo.msg
  *
  * See mavlink message documentation here:
  * https://mavlink.io/en/messages/common.html#ESC_INFO
 */
typedef struct mavros_msgs__msg__ESCInfo
{
  std_msgs__msg__Header header;
  uint16_t counter;
  uint8_t count;
  uint8_t connection_type;
  uint8_t info;
  mavros_msgs__msg__ESCInfoItem__Sequence esc_info;
} mavros_msgs__msg__ESCInfo;

// Struct for a sequence of mavros_msgs__msg__ESCInfo.
typedef struct mavros_msgs__msg__ESCInfo__Sequence
{
  mavros_msgs__msg__ESCInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__ESCInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_INFO__STRUCT_H_
