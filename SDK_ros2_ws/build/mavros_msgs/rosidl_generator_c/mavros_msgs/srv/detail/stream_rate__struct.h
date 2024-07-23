// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:srv/StreamRate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__STRUCT_H_
#define MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STREAM_ALL'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_ALL = 0
};

/// Constant 'STREAM_RAW_SENSORS'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_RAW_SENSORS = 1
};

/// Constant 'STREAM_EXTENDED_STATUS'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_EXTENDED_STATUS = 2
};

/// Constant 'STREAM_RC_CHANNELS'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_RC_CHANNELS = 3
};

/// Constant 'STREAM_RAW_CONTROLLER'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_RAW_CONTROLLER = 4
};

/// Constant 'STREAM_POSITION'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_POSITION = 6
};

/// Constant 'STREAM_EXTRA1'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_EXTRA1 = 10
};

/// Constant 'STREAM_EXTRA2'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_EXTRA2 = 11
};

/// Constant 'STREAM_EXTRA3'.
enum
{
  mavros_msgs__srv__StreamRate_Request__STREAM_EXTRA3 = 12
};

/// Struct defined in srv/StreamRate in the package mavros_msgs.
typedef struct mavros_msgs__srv__StreamRate_Request
{
  uint8_t stream_id;
  uint16_t message_rate;
  bool on_off;
} mavros_msgs__srv__StreamRate_Request;

// Struct for a sequence of mavros_msgs__srv__StreamRate_Request.
typedef struct mavros_msgs__srv__StreamRate_Request__Sequence
{
  mavros_msgs__srv__StreamRate_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__StreamRate_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/StreamRate in the package mavros_msgs.
typedef struct mavros_msgs__srv__StreamRate_Response
{
  uint8_t structure_needs_at_least_one_member;
} mavros_msgs__srv__StreamRate_Response;

// Struct for a sequence of mavros_msgs__srv__StreamRate_Response.
typedef struct mavros_msgs__srv__StreamRate_Response__Sequence
{
  mavros_msgs__srv__StreamRate_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__srv__StreamRate_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__STRUCT_H_
