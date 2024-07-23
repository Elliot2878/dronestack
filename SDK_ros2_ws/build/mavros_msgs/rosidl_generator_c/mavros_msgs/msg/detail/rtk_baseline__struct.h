// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/RTKBaseline.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'RTK_BASELINE_COORDINATE_SYSTEM_ECEF'.
/**
  * Earth-centered, earth-fixed
 */
enum
{
  mavros_msgs__msg__RTKBaseline__RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0
};

/// Constant 'RTK_BASELINE_COORDINATE_SYSTEM_NED'.
/**
  * RTK basestation centered, north, east, down
 */
enum
{
  mavros_msgs__msg__RTKBaseline__RTK_BASELINE_COORDINATE_SYSTEM_NED = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/RTKBaseline in the package mavros_msgs.
/**
  * RTKBaseline received from the FCU.
  * Full description: https://mavlink.io/en/messages/common.html#GPS_RTK
  * Mavlink Common, #127and #128
 */
typedef struct mavros_msgs__msg__RTKBaseline
{
  std_msgs__msg__Header header;
  uint32_t time_last_baseline_ms;
  uint8_t rtk_receiver_id;
  uint16_t wn;
  uint32_t tow;
  uint8_t rtk_health;
  uint8_t rtk_rate;
  uint8_t nsats;
  uint8_t baseline_coords_type;
  int32_t baseline_a_mm;
  int32_t baseline_b_mm;
  int32_t baseline_c_mm;
  uint32_t accuracy;
  int32_t iar_num_hypotheses;
} mavros_msgs__msg__RTKBaseline;

// Struct for a sequence of mavros_msgs__msg__RTKBaseline.
typedef struct mavros_msgs__msg__RTKBaseline__Sequence
{
  mavros_msgs__msg__RTKBaseline * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__RTKBaseline__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__STRUCT_H_
