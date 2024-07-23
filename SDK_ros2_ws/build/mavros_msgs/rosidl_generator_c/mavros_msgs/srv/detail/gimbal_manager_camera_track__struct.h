// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/GimbalManagerCameraTrack.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CAMERA_TRACK_MODE_POINT'.
/**
  * CAMERA_TRACK_MODE
  *  If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking.
 */
enum
{
  mavros_msgs__srv__GimbalManagerCameraTrack_Request__CAMERA_TRACK_MODE_POINT = 0
};

/// Constant 'CAMERA_TRACK_MODE_RECTANGLE'.
/**
  * If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking.
 */
enum
{
  mavros_msgs__srv__GimbalManagerCameraTrack_Request__CAMERA_TRACK_MODE_RECTANGLE = 1
};

/// Constant 'CAMERA_TRACK_MODE_STOP_TRACKING'.
/**
  * Stops ongoing tracking.
 */
enum
{
  mavros_msgs__srv__GimbalManagerCameraTrack_Request__CAMERA_TRACK_MODE_STOP_TRACKING = 2
};

/// Struct defined in srv/GimbalManagerCameraTrack in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerCameraTrack_Request
{
  /// enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE
  uint8_t mode;
  /// For CAMERA_TRACK_POINT
  ///  Point to track x value (normalized 0..1, 0 is left, 1 is right).
  float x;
  /// Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
  float y;
  /// Point radius (normalized 0..1, 0 is image left, 1 is image right).
  float radius;
  /// For CAMERA_TRACK_RECTANGLE
  ///  Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
  float top_left_x;
  /// Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
  float top_left_y;
  /// Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
  float bottom_right_x;
  /// Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
  float bottom_right_y;
} mavros_msgs__srv__GimbalManagerCameraTrack_Request;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerCameraTrack_Request.
typedef struct mavros_msgs__srv__GimbalManagerCameraTrack_Request__Sequence
{
  mavros_msgs__srv__GimbalManagerCameraTrack_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerCameraTrack_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GimbalManagerCameraTrack in the package mavros_msgs.
typedef struct mavros_msgs__srv__GimbalManagerCameraTrack_Response
{
  bool success;
  /// raw result returned by COMMAND_ACK
  uint8_t result;
} mavros_msgs__srv__GimbalManagerCameraTrack_Response;

// Struct for a sequence of mavros_msgs__srv__GimbalManagerCameraTrack_Response.
typedef struct mavros_msgs__srv__GimbalManagerCameraTrack_Response__Sequence
{
  mavros_msgs__srv__GimbalManagerCameraTrack_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__GimbalManagerCameraTrack_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__STRUCT_H_
