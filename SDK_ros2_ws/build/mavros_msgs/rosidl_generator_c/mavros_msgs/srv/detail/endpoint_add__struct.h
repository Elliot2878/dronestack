// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/EndpointAdd.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_FCU'.
enum
{
  mavros_msgs__srv__EndpointAdd_Request__TYPE_FCU = 0
};

/// Constant 'TYPE_GCS'.
enum
{
  mavros_msgs__srv__EndpointAdd_Request__TYPE_GCS = 1
};

/// Constant 'TYPE_UAS'.
enum
{
  mavros_msgs__srv__EndpointAdd_Request__TYPE_UAS = 2
};

// Include directives for member types
// Member 'url'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/EndpointAdd in the package mavros_msgs.
typedef struct mavros_msgs__srv__EndpointAdd_Request
{
  /// mavconn URL or topic prefix for TYPE_UAS
  rosidl_runtime_c__String url;
  /// should be set to one of the TYPE_xxx
  uint8_t type;
} mavros_msgs__srv__EndpointAdd_Request;

// Struct for a sequence of mavros_msgs__srv__EndpointAdd_Request.
typedef struct mavros_msgs__srv__EndpointAdd_Request__Sequence
{
  mavros_msgs__srv__EndpointAdd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__EndpointAdd_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/EndpointAdd in the package mavros_msgs.
typedef struct mavros_msgs__srv__EndpointAdd_Response
{
  /// true if endpoint added and opened
  bool successful;
  /// returns error description if open fails
  rosidl_runtime_c__String reason;
  /// ID of new endpoint, should be > 0 if endpoint created
  uint32_t id;
} mavros_msgs__srv__EndpointAdd_Response;

// Struct for a sequence of mavros_msgs__srv__EndpointAdd_Response.
typedef struct mavros_msgs__srv__EndpointAdd_Response__Sequence
{
  mavros_msgs__srv__EndpointAdd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__EndpointAdd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__STRUCT_H_
