// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/TerrainReport.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__STRUCT_H_

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

/// Struct defined in msg/TerrainReport in the package mavros_msgs.
/**
  * Message for TERRAIN_REPORT
  * https://mavlink.io/en/messages/common.html#TERRAIN_REPORT
 */
typedef struct mavros_msgs__msg__TerrainReport
{
  std_msgs__msg__Header header;
  double latitude;
  double longitude;
  uint16_t spacing;
  /// in meters, terrain height
  float terrain_height;
  /// in meters, vehicle height above terrain
  float current_height;
  uint16_t pending;
  uint16_t loaded;
} mavros_msgs__msg__TerrainReport;

// Struct for a sequence of mavros_msgs__msg__TerrainReport.
typedef struct mavros_msgs__msg__TerrainReport__Sequence
{
  mavros_msgs__msg__TerrainReport * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__TerrainReport__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__STRUCT_H_
