// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/LandingTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GLOBAL'.
/**
  * MAV_FRAME enum
  * Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
 */
enum
{
  mavros_msgs__msg__LandingTarget__GLOBAL = 0
};

/// Constant 'LOCAL_NED'.
/**
  * Local coordinate frame, Z-up (x: north, y: east, z: down).
 */
enum
{
  mavros_msgs__msg__LandingTarget__LOCAL_NED = 2
};

/// Constant 'MISSION'.
/**
  * NOT a coordinate frame, indicates a mission command.
 */
enum
{
  mavros_msgs__msg__LandingTarget__MISSION = 3
};

/// Constant 'GLOBAL_RELATIVE_ALT'.
/**
  * Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
 */
enum
{
  mavros_msgs__msg__LandingTarget__GLOBAL_RELATIVE_ALT = 4
};

/// Constant 'LOCAL_ENU'.
/**
  * Local coordinate frame, Z-down (x: east, y: north, z: up)
 */
enum
{
  mavros_msgs__msg__LandingTarget__LOCAL_ENU = 5
};

/// Constant 'GLOBAL_INT'.
/**
  * Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
 */
enum
{
  mavros_msgs__msg__LandingTarget__GLOBAL_INT = 6
};

/// Constant 'GLOBAL_RELATIVE_ALT_INT'.
/**
  * Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
 */
enum
{
  mavros_msgs__msg__LandingTarget__GLOBAL_RELATIVE_ALT_INT = 7
};

/// Constant 'LOCAL_OFFSET_NED'.
/**
  * Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
 */
enum
{
  mavros_msgs__msg__LandingTarget__LOCAL_OFFSET_NED = 8
};

/// Constant 'BODY_NED'.
/**
  * Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
 */
enum
{
  mavros_msgs__msg__LandingTarget__BODY_NED = 9
};

/// Constant 'BODY_OFFSET_NED'.
/**
  * Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
 */
enum
{
  mavros_msgs__msg__LandingTarget__BODY_OFFSET_NED = 10
};

/// Constant 'GLOBAL_TERRAIN_ALT'.
/**
  * Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
 */
enum
{
  mavros_msgs__msg__LandingTarget__GLOBAL_TERRAIN_ALT = 11
};

/// Constant 'GLOBAL_TERRAIN_ALT_INT'.
/**
  * Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
 */
enum
{
  mavros_msgs__msg__LandingTarget__GLOBAL_TERRAIN_ALT_INT = 12
};

/// Constant 'LIGHT_BEACON'.
/**
  * LANDING_TARGET_TYPE enum
  * Landing target signaled by light beacon (ex: IR-LOCK)
 */
enum
{
  mavros_msgs__msg__LandingTarget__LIGHT_BEACON = 0
};

/// Constant 'RADIO_BEACON'.
/**
  * Landing target signaled by radio beacon (ex: ILS, NDB)
 */
enum
{
  mavros_msgs__msg__LandingTarget__RADIO_BEACON = 1
};

/// Constant 'VISION_FIDUCIAL'.
/**
  * Landing target represented by a fiducial marker (ex: ARTag)
 */
enum
{
  mavros_msgs__msg__LandingTarget__VISION_FIDUCIAL = 2
};

/// Constant 'VISION_OTHER'.
/**
  * Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
 */
enum
{
  mavros_msgs__msg__LandingTarget__VISION_OTHER = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/LandingTarget in the package mavros_msgs.
/**
  * MAVLink message: LANDING_TARGET
  * https://mavlink.io/en/messages/common.html
 */
typedef struct mavros_msgs__msg__LandingTarget
{
  std_msgs__msg__Header header;
  uint8_t target_num;
  uint8_t frame;
  float angle[2];
  float distance;
  float size[2];
  geometry_msgs__msg__Pose pose;
  uint8_t type;
} mavros_msgs__msg__LandingTarget;

// Struct for a sequence of mavros_msgs__msg__LandingTarget.
typedef struct mavros_msgs__msg__LandingTarget__Sequence
{
  mavros_msgs__msg__LandingTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__LandingTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__STRUCT_H_
