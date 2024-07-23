// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/Tunnel.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TUNNEL__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__TUNNEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PAYLOAD_TYPE_UNKNOWN'.
/**
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum('MAV_TUNNEL_PAYLOAD_TYPE', 'PAYLOAD_TYPE_', 16)
  * ]]]
  * MAV_TUNNEL_PAYLOAD_TYPE
  * Encoding of payload unknown.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_UNKNOWN = 0
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED0'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED0 = 200
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED1'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED1 = 201
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED2'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED2 = 202
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED3'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED3 = 203
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED4'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED4 = 204
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED5'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED5 = 205
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED6'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED6 = 206
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED7'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED7 = 207
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED8'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED8 = 208
};

/// Constant 'PAYLOAD_TYPE_STORM32_RESERVED9'.
/**
  * Registered for STorM32 gimbal controller.
 */
enum
{
  mavros_msgs__msg__Tunnel__PAYLOAD_TYPE_STORM32_RESERVED9 = 209
};

/// Struct defined in msg/Tunnel in the package mavros_msgs.
/**
  * Tunnel
  *
  * https://mavlink.io/en/messages/common.html#TUNNEL
 */
typedef struct mavros_msgs__msg__Tunnel
{
  uint8_t target_system;
  uint8_t target_component;
  uint16_t payload_type;
  uint8_t payload_length;
  uint8_t payload[128];
} mavros_msgs__msg__Tunnel;

// Struct for a sequence of mavros_msgs__msg__Tunnel.
typedef struct mavros_msgs__msg__Tunnel__Sequence
{
  mavros_msgs__msg__Tunnel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__Tunnel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__TUNNEL__STRUCT_H_
