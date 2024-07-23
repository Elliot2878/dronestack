// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/VehicleInfoGet.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GET_MY_SYSID'.
enum
{
  mavros_msgs__srv__VehicleInfoGet_Request__GET_MY_SYSID = 0
};

/// Constant 'GET_MY_COMPID'.
enum
{
  mavros_msgs__srv__VehicleInfoGet_Request__GET_MY_COMPID = 0
};

/// Struct defined in srv/VehicleInfoGet in the package mavros_msgs.
typedef struct mavros_msgs__srv__VehicleInfoGet_Request
{
  uint8_t sysid;
  uint8_t compid;
  bool get_all;
} mavros_msgs__srv__VehicleInfoGet_Request;

// Struct for a sequence of mavros_msgs__srv__VehicleInfoGet_Request.
typedef struct mavros_msgs__srv__VehicleInfoGet_Request__Sequence
{
  mavros_msgs__srv__VehicleInfoGet_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__VehicleInfoGet_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'vehicles'
#include "mavros_msgs/msg/detail/vehicle_info__struct.h"

/// Struct defined in srv/VehicleInfoGet in the package mavros_msgs.
typedef struct mavros_msgs__srv__VehicleInfoGet_Response
{
  bool success;
  mavros_msgs__msg__VehicleInfo__Sequence vehicles;
} mavros_msgs__srv__VehicleInfoGet_Response;

// Struct for a sequence of mavros_msgs__srv__VehicleInfoGet_Response.
typedef struct mavros_msgs__srv__VehicleInfoGet_Response__Sequence
{
  mavros_msgs__srv__VehicleInfoGet_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__VehicleInfoGet_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__STRUCT_H_
