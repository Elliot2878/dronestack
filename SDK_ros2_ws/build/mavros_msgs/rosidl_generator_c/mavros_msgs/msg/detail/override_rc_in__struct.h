// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OverrideRCIn.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CHAN_RELEASE'.
/**
  * https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
 */
enum
{
  mavros_msgs__msg__OverrideRCIn__CHAN_RELEASE = 0
};

/// Constant 'CHAN_NOCHANGE'.
enum
{
  mavros_msgs__msg__OverrideRCIn__CHAN_NOCHANGE = 65535
};

/// Struct defined in msg/OverrideRCIn in the package mavros_msgs.
/**
  * Override RC Input
  * Currently MAVLink defines override for 18 channels
 */
typedef struct mavros_msgs__msg__OverrideRCIn
{
  uint16_t channels[18];
} mavros_msgs__msg__OverrideRCIn;

// Struct for a sequence of mavros_msgs__msg__OverrideRCIn.
typedef struct mavros_msgs__msg__OverrideRCIn__Sequence
{
  mavros_msgs__msg__OverrideRCIn * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OverrideRCIn__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__STRUCT_H_
