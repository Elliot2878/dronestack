// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/SetMavFrame.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRAME_GLOBAL'.
/**
  * XXX DEPRECATED
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum('MAV_FRAME', 'FRAME_')
  * ]]]
  * MAV_FRAME
  * Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_GLOBAL = 0
};

/// Constant 'FRAME_LOCAL_NED'.
/**
  * NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_LOCAL_NED = 1
};

/// Constant 'FRAME_MISSION'.
/**
  * NOT a coordinate frame, indicates a mission command.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_MISSION = 2
};

/// Constant 'FRAME_GLOBAL_RELATIVE_ALT'.
/**
  * Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_GLOBAL_RELATIVE_ALT = 3
};

/// Constant 'FRAME_LOCAL_ENU'.
/**
  * ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_LOCAL_ENU = 4
};

/// Constant 'FRAME_GLOBAL_INT'.
/**
  * Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_GLOBAL_INT = 5
};

/// Constant 'FRAME_GLOBAL_RELATIVE_ALT_INT'.
/**
  * Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home location.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_GLOBAL_RELATIVE_ALT_INT = 6
};

/// Constant 'FRAME_LOCAL_OFFSET_NED'.
/**
  * NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_LOCAL_OFFSET_NED = 7
};

/// Constant 'FRAME_BODY_NED'.
/**
  * Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/accelaration values.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_BODY_NED = 8
};

/// Constant 'FRAME_BODY_OFFSET_NED'.
/**
  * This is the same as MAV_FRAME_BODY_FRD.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_BODY_OFFSET_NED = 9
};

/// Constant 'FRAME_GLOBAL_TERRAIN_ALT'.
/**
  * Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_GLOBAL_TERRAIN_ALT = 10
};

/// Constant 'FRAME_GLOBAL_TERRAIN_ALT_INT'.
/**
  * Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_GLOBAL_TERRAIN_ALT_INT = 11
};

/// Constant 'FRAME_BODY_FRD'.
/**
  * FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The forward axis is aligned to the front of the vehicle in the horizontal plane.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_BODY_FRD = 12
};

/// Constant 'FRAME_RESERVED_13'.
/**
  * MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_13 = 13
};

/// Constant 'FRAME_RESERVED_14'.
/**
  * MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_14 = 14
};

/// Constant 'FRAME_RESERVED_15'.
/**
  * MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_15 = 15
};

/// Constant 'FRAME_RESERVED_16'.
/**
  * MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_16 = 16
};

/// Constant 'FRAME_RESERVED_17'.
/**
  * MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_17 = 17
};

/// Constant 'FRAME_RESERVED_18'.
/**
  * MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_18 = 18
};

/// Constant 'FRAME_RESERVED_19'.
/**
  * MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_RESERVED_19 = 19
};

/// Constant 'FRAME_LOCAL_FRD'.
/**
  * FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_LOCAL_FRD = 20
};

/// Constant 'FRAME_LOCAL_FLU'.
/**
  * FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
 */
enum
{
  mavros_msgs__srv__SetMavFrame_Request__FRAME_LOCAL_FLU = 21
};

/// Struct defined in srv/SetMavFrame in the package mavros_msgs.
typedef struct mavros_msgs__srv__SetMavFrame_Request
{
  /// ]] (checksum: c5ddb537c91e87c4efba8b24c9cde50e)
  uint8_t mav_frame;
} mavros_msgs__srv__SetMavFrame_Request;

// Struct for a sequence of mavros_msgs__srv__SetMavFrame_Request.
typedef struct mavros_msgs__srv__SetMavFrame_Request__Sequence
{
  mavros_msgs__srv__SetMavFrame_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__SetMavFrame_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetMavFrame in the package mavros_msgs.
typedef struct mavros_msgs__srv__SetMavFrame_Response
{
  bool success;
} mavros_msgs__srv__SetMavFrame_Response;

// Struct for a sequence of mavros_msgs__srv__SetMavFrame_Response.
typedef struct mavros_msgs__srv__SetMavFrame_Response__Sequence
{
  mavros_msgs__srv__SetMavFrame_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__SetMavFrame_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__STRUCT_H_
