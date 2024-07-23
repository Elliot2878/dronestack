// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/RadioStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__STRUCT_H_

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

/// Struct defined in msg/RadioStatus in the package mavros_msgs.
/**
  * RADIO_STATUS message
 */
typedef struct mavros_msgs__msg__RadioStatus
{
  std_msgs__msg__Header header;
  /// message data
  uint8_t rssi;
  uint8_t remrssi;
  uint8_t txbuf;
  uint8_t noise;
  uint8_t remnoise;
  uint16_t rxerrors;
  uint16_t fixed;
  /// calculated
  float rssi_dbm;
  float remrssi_dbm;
} mavros_msgs__msg__RadioStatus;

// Struct for a sequence of mavros_msgs__msg__RadioStatus.
typedef struct mavros_msgs__msg__RadioStatus__Sequence
{
  mavros_msgs__msg__RadioStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__RadioStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__STRUCT_H_
