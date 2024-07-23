// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GPSRTK.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSRTK__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GPSRTK__STRUCT_H_

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

/// Struct defined in msg/GPSRTK in the package mavros_msgs.
/**
  * FCU GPS RTK message for the gps_status plugin
  * A copy of <a href="https://mavlink.io/en/messages/common.html#GPS_RTK">mavlink GPS_RTK message</a>
 */
typedef struct mavros_msgs__msg__GPSRTK
{
  std_msgs__msg__Header header;
  /// Identification of connected RTK receiver.
  uint8_t rtk_receiver_id;
  /// GPS Week Number of last baseline.
  int16_t wn;
  /// GPS Time of Week of last baseline.
  uint32_t tow;
  /// GPS-specific health report for RTK data.
  uint8_t rtk_health;
  /// Rate of baseline messages being received by GPS.
  uint8_t rtk_rate;
  /// Current number of sats used for RTK calculation.
  uint8_t nsats;
  /// Current baseline in ECEF x or NED north component, depends on header.frame_id.
  int32_t baseline_a;
  /// Current baseline in ECEF y or NED east component, depends on header.frame_id.
  int32_t baseline_b;
  /// Current baseline in ECEF z or NED down component, depends on header.frame_id.
  int32_t baseline_c;
  /// Current estimate of baseline accuracy.
  uint32_t accuracy;
  /// Current number of integer ambiguity hypotheses.
  int32_t iar_num_hypotheses;
} mavros_msgs__msg__GPSRTK;

// Struct for a sequence of mavros_msgs__msg__GPSRTK.
typedef struct mavros_msgs__msg__GPSRTK__Sequence
{
  mavros_msgs__msg__GPSRTK * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GPSRTK__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSRTK__STRUCT_H_
