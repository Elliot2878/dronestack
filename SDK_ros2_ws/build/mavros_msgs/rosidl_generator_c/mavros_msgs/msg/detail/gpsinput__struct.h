// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GPSINPUT.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSINPUT__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GPSINPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GPS_FIX_TYPE_NO_GPS'.
/**
  * GPS_FIX_TYPE enum
  * No GPS connected
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_NO_GPS = 0
};

/// Constant 'GPS_FIX_TYPE_NO_FIX'.
/**
  * No position information, GPS is connected
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_NO_FIX = 1
};

/// Constant 'GPS_FIX_TYPE_2D_FIX'.
/**
  * 2D position
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_2D_FIX = 2
};

/// Constant 'GPS_FIX_TYPE_3D_FIX'.
/**
  * 3D position
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_3D_FIX = 3
};

/// Constant 'GPS_FIX_TYPE_DGPS'.
/**
  * DGPS/SBAS aided 3D position
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_DGPS = 4
};

/// Constant 'GPS_FIX_TYPE_RTK_FLOATR'.
/**
  * TK float, 3D position
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_RTK_FLOATR = 5
};

/// Constant 'GPS_FIX_TYPE_RTK_FIXEDR'.
/**
  * TK Fixed, 3D position
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_RTK_FIXEDR = 6
};

/// Constant 'GPS_FIX_TYPE_STATIC'.
/**
  * Static fixed, typically used for base stations
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_STATIC = 7
};

/// Constant 'GPS_FIX_TYPE_PPP'.
/**
  * PPP, 3D position
 */
enum
{
  mavros_msgs__msg__GPSINPUT__GPS_FIX_TYPE_PPP = 8
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GPSINPUT in the package mavros_msgs.
/**
  * FCU GPS INPUT message for the gps_input plugin
  * <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">mavlink GPS_INPUT message</a>.
 */
typedef struct mavros_msgs__msg__GPSINPUT
{
  std_msgs__msg__Header header;
  /// GPS fix type
  uint8_t fix_type;
  /// ID of the GPS for multiple GPS inputs
  uint8_t gps_id;
  /// Bitmap indicating which GPS input flags fields to ignore. All other fields must be provided.
  uint16_t ignore_flags;
  /// GPS time (from start of GPS week)
  uint32_t time_week_ms;
  /// GPS week number
  uint16_t time_week;
  /// Latitude (WGS84, EGM96 ellipsoid)
  int32_t lat;
  /// Longitude (WGS84, EGM96 ellipsoid)
  int32_t lon;
  /// Altitude (MSL). Positive for up.
  float alt;
  /// GPS HDOP horizontal dilution of position.
  float hdop;
  /// GPS VDOP vertical dilution of position
  float vdop;
  /// GPS velocity in NORTH direction in earth-fixed NED frame
  float vn;
  /// GPS velocity in EAST direction in earth-fixed NED frame
  float ve;
  /// GPS velocity in DOWN direction in earth-fixed NED frame
  float vd;
  /// GPS speed accuracy
  float speed_accuracy;
  /// GPS horizontal accuracy
  float horiz_accuracy;
  /// GPS vertical accuracy
  float vert_accuracy;
  /// Number of satellites visible. If unknown, set to 255
  uint8_t satellites_visible;
  /// Yaw in earth frame from north.
  uint16_t yaw;
} mavros_msgs__msg__GPSINPUT;

// Struct for a sequence of mavros_msgs__msg__GPSINPUT.
typedef struct mavros_msgs__msg__GPSINPUT__Sequence
{
  mavros_msgs__msg__GPSINPUT * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GPSINPUT__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSINPUT__STRUCT_H_
