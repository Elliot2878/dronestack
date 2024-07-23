// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/CommandTOLLocal.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/CommandTOLLocal in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandTOLLocal_Request
{
  /// used by takeoff
  float min_pitch;
  /// used by land (landing position accuracy)
  float offset;
  /// speed of takeoff/land in m/s
  float rate;
  /// in radians
  float yaw;
  /// (x,y,z) in meters
  geometry_msgs__msg__Vector3 position;
} mavros_msgs__srv__CommandTOLLocal_Request;

// Struct for a sequence of mavros_msgs__srv__CommandTOLLocal_Request.
typedef struct mavros_msgs__srv__CommandTOLLocal_Request__Sequence
{
  mavros_msgs__srv__CommandTOLLocal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandTOLLocal_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CommandTOLLocal in the package mavros_msgs.
typedef struct mavros_msgs__srv__CommandTOLLocal_Response
{
  bool success;
  uint8_t result;
} mavros_msgs__srv__CommandTOLLocal_Response;

// Struct for a sequence of mavros_msgs__srv__CommandTOLLocal_Response.
typedef struct mavros_msgs__srv__CommandTOLLocal_Response__Sequence
{
  mavros_msgs__srv__CommandTOLLocal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__CommandTOLLocal_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__STRUCT_H_
