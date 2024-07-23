// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/TimesyncStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__STRUCT_H_

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

/// Struct defined in msg/TimesyncStatus in the package mavros_msgs.
/**
  * Status of the MAVLink time synchroniser
 */
typedef struct mavros_msgs__msg__TimesyncStatus
{
  std_msgs__msg__Header header;
  /// remote system timestamp (nanoseconds)
  uint64_t remote_timestamp_ns;
  /// raw time offset directly observed from this timesync packet (nanoseconds)
  int64_t observed_offset_ns;
  /// smoothed time offset between companion system and Mavros (nanoseconds)
  int64_t estimated_offset_ns;
  /// round trip time of this timesync packet (milliseconds)
  float round_trip_time_ms;
} mavros_msgs__msg__TimesyncStatus;

// Struct for a sequence of mavros_msgs__msg__TimesyncStatus.
typedef struct mavros_msgs__msg__TimesyncStatus__Sequence
{
  mavros_msgs__msg__TimesyncStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__TimesyncStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__STRUCT_H_
