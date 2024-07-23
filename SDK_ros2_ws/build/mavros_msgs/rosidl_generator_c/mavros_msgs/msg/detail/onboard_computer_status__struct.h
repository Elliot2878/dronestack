// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/OnboardComputerStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ONBOARD_COMPUTER_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ONBOARD_COMPUTER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/OnboardComputerStatus in the package mavros_msgs.
/**
  * Mavros message: ONBOARDCOMPUTERSTATUS
 */
typedef struct mavros_msgs__msg__OnboardComputerStatus
{
  std_msgs__msg__Header header;
  /// See enum MAV_COMPONENT
  uint8_t component;
  /// Time since system boot
  uint32_t uptime;
  /// Type of the onboard computer: 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.
  uint8_t type;
  /// CPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused.
  uint8_t cpu_cores[8];
  /// Combined CPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field is unused
  uint8_t cpu_combined[10];
  /// GPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused
  uint8_t gpu_cores[4];
  /// Combined GPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field is unused.
  uint8_t gpu_combined[10];
  /// Temperature of the board. A value of INT8_MAX implies the field is unused.
  int8_t temperature_board;
  /// Temperature of the CPU core. A value of INT8_MAX implies the field is unused.
  int8_t temperature_core[8];
  /// Fan speeds. A value of INT16_MAX implies the field is unused.
  int16_t fan_speed[4];
  /// Amount of used RAM on the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t ram_usage;
  /// Total amount of RAM on the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t ram_total;
  /// Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable). A value of UINT32_MAX implies the field is unused.
  uint32_t storage_type[4];
  /// Amount of used storage space on the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t storage_usage[4];
  /// Total amount of storage space on the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t storage_total[4];
  /// Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary.
  uint32_t link_type[6];
  /// Network traffic from the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t link_tx_rate[6];
  /// Network traffic to the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t link_rx_rate[6];
  /// Network capacity from the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t link_tx_max[6];
  /// Network capacity to the component system. A value of UINT32_MAX implies the field is unused.
  uint32_t link_rx_max[6];
} mavros_msgs__msg__OnboardComputerStatus;

// Struct for a sequence of mavros_msgs__msg__OnboardComputerStatus.
typedef struct mavros_msgs__msg__OnboardComputerStatus__Sequence
{
  mavros_msgs__msg__OnboardComputerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__OnboardComputerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ONBOARD_COMPUTER_STATUS__STRUCT_H_
