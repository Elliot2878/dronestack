// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/PlayTuneV2.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'QBASIC1_1'.
/**
  * TUNE_FORMAT enum
 */
enum
{
  mavros_msgs__msg__PlayTuneV2__QBASIC1_1 = 1
};

/// Constant 'MML_MODERN'.
enum
{
  mavros_msgs__msg__PlayTuneV2__MML_MODERN = 2
};

// Include directives for member types
// Member 'tune'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PlayTuneV2 in the package mavros_msgs.
/**
  * Play tune V2
  *
  * https://mavlink.io/en/messages/common.html#PLAY_TUNE_V2
 */
typedef struct mavros_msgs__msg__PlayTuneV2
{
  uint8_t format;
  rosidl_runtime_c__String tune;
} mavros_msgs__msg__PlayTuneV2;

// Struct for a sequence of mavros_msgs__msg__PlayTuneV2.
typedef struct mavros_msgs__msg__PlayTuneV2__Sequence
{
  mavros_msgs__msg__PlayTuneV2 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__PlayTuneV2__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__STRUCT_H_
