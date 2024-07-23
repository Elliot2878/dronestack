// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__STRUCT_H_

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

/// Struct defined in msg/EstimatorStatus in the package mavros_msgs.
/**
  * Current autopilot estimator state
  *
  * https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS_FLAGS
 */
typedef struct mavros_msgs__msg__EstimatorStatus
{
  std_msgs__msg__Header header;
  bool attitude_status_flag;
  bool velocity_horiz_status_flag;
  bool velocity_vert_status_flag;
  bool pos_horiz_rel_status_flag;
  bool pos_horiz_abs_status_flag;
  bool pos_vert_abs_status_flag;
  bool pos_vert_agl_status_flag;
  bool const_pos_mode_status_flag;
  bool pred_pos_horiz_rel_status_flag;
  bool pred_pos_horiz_abs_status_flag;
  bool gps_glitch_status_flag;
  bool accel_error_status_flag;
} mavros_msgs__msg__EstimatorStatus;

// Struct for a sequence of mavros_msgs__msg__EstimatorStatus.
typedef struct mavros_msgs__msg__EstimatorStatus__Sequence
{
  mavros_msgs__msg__EstimatorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__EstimatorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__STRUCT_H_
