// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/LogData.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LOG_DATA__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__LOG_DATA__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LogData in the package mavros_msgs.
/**
  * Reply to LogRequestData, - a chunk of a log
  *
  *  :id: - log id
  *  :offset: - offset into the log
  *  :data: - chunk of data (if zero-sized, then there are no more chunks)
 */
typedef struct mavros_msgs__msg__LogData
{
  std_msgs__msg__Header header;
  uint16_t id;
  uint32_t offset;
  rosidl_runtime_c__uint8__Sequence data;
} mavros_msgs__msg__LogData;

// Struct for a sequence of mavros_msgs__msg__LogData.
typedef struct mavros_msgs__msg__LogData__Sequence
{
  mavros_msgs__msg__LogData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__LogData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__LOG_DATA__STRUCT_H_
