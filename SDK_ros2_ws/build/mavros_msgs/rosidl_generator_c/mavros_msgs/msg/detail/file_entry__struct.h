// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/FileEntry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__FILE_ENTRY__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__FILE_ENTRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_FILE'.
enum
{
  mavros_msgs__msg__FileEntry__TYPE_FILE = 0
};

/// Constant 'TYPE_DIRECTORY'.
enum
{
  mavros_msgs__msg__FileEntry__TYPE_DIRECTORY = 1
};

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FileEntry in the package mavros_msgs.
/**
  * File/Dir information
 */
typedef struct mavros_msgs__msg__FileEntry
{
  rosidl_runtime_c__String name;
  uint8_t type;
  uint64_t size;
} mavros_msgs__msg__FileEntry;

// Struct for a sequence of mavros_msgs__msg__FileEntry.
typedef struct mavros_msgs__msg__FileEntry__Sequence
{
  mavros_msgs__msg__FileEntry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__FileEntry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__FILE_ENTRY__STRUCT_H_
