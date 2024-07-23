// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/GimbalManagerSetRoi.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ROI_MODE_LOCATION'.
/**
  * ROI_MODE
  *  Sets the region of interest (ROI) to a location.
 */
enum
{
  mavros_msgs__srv__GimbalManagerSetRoi_Request__ROI_MODE_LOCATION = 0
};

/// Constant 'ROI_MODE_WP_NEXT_OFFSET'.
/**
  * Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset.
 */
enum
{
  mavros_msgs__srv__GimbalManagerSetRoi_Request__ROI_MODE_WP_NEXT_OFFSET = 1
};

/// Constant 'ROI_MODE_SYSID'.
/**
  * Mount tracks system with specified system ID
 */
enum
{
  mavros_msgs__srv__GimbalManagerSetRoi_Request__ROI_MODE_SYSID = 2
};

/// Constant 'ROI_MODE_NONE'.
/**
  * Cancels any previous ROI setting and returns vehicle to defaults
 */
enum
{
  mavros_msgs__srv__GimbalManagerSetRoi_Request__ROI_MODE_NONE = 3
};

/// Struct defined in srv/GimbalManagerSetRoi in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerSetRoi_Request
{
  /// enumerator to indicate ROI mode setting - see ROI_MODE
  uint8_t mode;
  /// Component ID of gimbal device to address
  /// (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
  /// components. Send command multiple times for more than
  /// one gimbal (but not all gimbals).  Default Mavlink gimbal
  /// device ids: 154, 171-175
  uint8_t gimbal_device_id;
  /// For ROI_MODE_LOCATION
  float latitude;
  float longitude;
  /// Meters
  float altitude;
  /// For ROI_MODE_WP_NEXT_OFFSET
  ///  Pitch offset from next waypoint, positive pitching up
  float pitch_offset;
  /// Roll offset from next waypoint, positive rolling to the right
  float roll_offset;
  /// Yaw offset from next waypoint, positive yawing to the right
  float yaw_offset;
  /// For ROI_MODE_SYSID
  ///  System ID to track (min: 1, max: 255)
  uint8_t sysid;
} mavros_msgs__srv__GimbalManagerSetRoi_Request;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerSetRoi_Request.
typedef struct mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence
{
  mavros_msgs__srv__GimbalManagerSetRoi_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerSetRoi_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GimbalManagerSetRoi in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerSetRoi_Response
{
  bool success;
  /// raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__GimbalManagerSetRoi_Response;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerSetRoi_Response.
typedef struct mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence
{
  mavros_msgs__srv__GimbalManagerSetRoi_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerSetRoi_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__STRUCT_H_
