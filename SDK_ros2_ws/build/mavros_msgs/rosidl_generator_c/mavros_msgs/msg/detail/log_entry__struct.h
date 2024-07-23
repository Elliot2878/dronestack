// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/LogEntry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__STRUCT_H_

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
// Member 'time_utc'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/LogEntry in the package mavros_msgs.
/**
  * Information about a single log
  *
  *  :id: - log id
  *  :num_logs: - total number of logs
  *  :last_log_num: - id of last log
  *  :time_utc: - UTC timestamp of log (ros::Time(0) if not available)
  *  :size: - size of log in bytes (may be approximate)
 */
typedef struct mavros_msgs__msg__LogEntry
{
  std_msgs__msg__Header header;
  uint16_t id;
  uint16_t num_logs;
  uint16_t last_log_num;
  builtin_interfaces__msg__Time time_utc;
  uint32_t size;
} mavros_msgs__msg__LogEntry;

// Struct for a sequence of mavros_msgs__msg__LogEntry.
typedef struct mavros_msgs__msg__LogEntry__Sequence
{
  mavros_msgs__msg__LogEntry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__LogEntry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__STRUCT_H_
