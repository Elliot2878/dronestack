// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/GPSRAW.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSRAW__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__GPSRAW__STRUCT_H_

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
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_NO_GPS = 0
};

/// Constant 'GPS_FIX_TYPE_NO_FIX'.
/**
  * No position information, GPS is connected
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_NO_FIX = 1
};

/// Constant 'GPS_FIX_TYPE_2D_FIX'.
/**
  * 2D position
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_2D_FIX = 2
};

/// Constant 'GPS_FIX_TYPE_3D_FIX'.
/**
  * 3D position
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_3D_FIX = 3
};

/// Constant 'GPS_FIX_TYPE_DGPS'.
/**
  * DGPS/SBAS aided 3D position
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_DGPS = 4
};

/// Constant 'GPS_FIX_TYPE_RTK_FLOAT'.
/**
  * RTK float, 3D position
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_RTK_FLOAT = 5
};

/// Constant 'GPS_FIX_TYPE_RTK_FIXED'.
/**
  * RTK Fixed, 3D position
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_RTK_FIXED = 6
};

/// Constant 'GPS_FIX_TYPE_STATIC'.
/**
  * Static fixed, typically used for base stations
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_STATIC = 7
};

/// Constant 'GPS_FIX_TYPE_PPP'.
/**
  * PPP, 3D position
 */
enum
{
  mavros_msgs__msg__GPSRAW__GPS_FIX_TYPE_PPP = 8
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GPSRAW in the package mavros_msgs.
/**
  * FCU GPS RAW message for the gps_status plugin
  * A merge of <a href="https://mavlink.io/en/messages/common.html#GPS_RAW_INT">mavlink GPS_RAW_INT</a> and
  * <a href="https://mavlink.io/en/messages/common.html#GPS2_RAW">mavlink GPS2_RAW</a> messages.
 */
typedef struct mavros_msgs__msg__GPSRAW
{
  std_msgs__msg__Header header;
  /// GPS fix type
  uint8_t fix_type;
  /// Latitude (WGS84, EGM96 ellipsoid)
  int32_t lat;
  /// Longitude (WGS84, EGM96 ellipsoid)
  int32_t lon;
  /// Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
  int32_t alt;
  /// GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
  uint16_t eph;
  /// GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
  uint16_t epv;
  /// GPS ground speed. If unknown, set to: UINT16_MAX
  uint16_t vel;
  /// Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
  uint16_t cog;
  /// Number of satellites visible. If unknown, set to 255
  uint8_t satellites_visible;
  /// -*- only available with MAVLink v2.0 and GPS_RAW_INT messages -*-
  /// [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
  int32_t alt_ellipsoid;
  /// Position uncertainty. Positive for up.
  uint32_t h_acc;
  /// Altitude uncertainty. Positive for up.
  uint32_t v_acc;
  /// Speed uncertainty. Positive for up.
  uint32_t vel_acc;
  /// Heading / track uncertainty
  int32_t hdg_acc;
  /// Yaw in earth frame from north.
  uint16_t yaw;
  /// -*- only available with MAVLink v2.0 and GPS2_RAW messages -*-
  /// Number of DGPS satellites
  uint8_t dgps_numch;
  /// Age of DGPS info
  uint32_t dgps_age;
} mavros_msgs__msg__GPSRAW;

// Struct for a sequence of mavros_msgs__msg__GPSRAW.
typedef struct mavros_msgs__msg__GPSRAW__Sequence
{
  mavros_msgs__msg__GPSRAW * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__GPSRAW__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSRAW__STRUCT_H_
