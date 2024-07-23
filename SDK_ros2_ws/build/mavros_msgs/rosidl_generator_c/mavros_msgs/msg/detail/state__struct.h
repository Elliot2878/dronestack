// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/State.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_APM_PLANE_MANUAL'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_MANUAL = "MANUAL";

/// Constant 'MODE_APM_PLANE_CIRCLE'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_CIRCLE = "CIRCLE";

/// Constant 'MODE_APM_PLANE_STABILIZE'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_STABILIZE = "STABILIZE";

/// Constant 'MODE_APM_PLANE_TRAINING'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_TRAINING = "TRAINING";

/// Constant 'MODE_APM_PLANE_ACRO'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_ACRO = "ACRO";

/// Constant 'MODE_APM_PLANE_FBWA'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_FBWA = "FBWA";

/// Constant 'MODE_APM_PLANE_FBWB'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_FBWB = "FBWB";

/// Constant 'MODE_APM_PLANE_CRUISE'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_CRUISE = "CRUISE";

/// Constant 'MODE_APM_PLANE_AUTOTUNE'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_AUTOTUNE = "AUTOTUNE";

/// Constant 'MODE_APM_PLANE_AUTO'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_AUTO = "AUTO";

/// Constant 'MODE_APM_PLANE_RTL'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_RTL = "RTL";

/// Constant 'MODE_APM_PLANE_LOITER'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_LOITER = "LOITER";

/// Constant 'MODE_APM_PLANE_LAND'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_LAND = "LAND";

/// Constant 'MODE_APM_PLANE_GUIDED'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_GUIDED = "GUIDED";

/// Constant 'MODE_APM_PLANE_INITIALISING'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_INITIALISING = "INITIALISING";

/// Constant 'MODE_APM_PLANE_QSTABILIZE'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_QSTABILIZE = "QSTABILIZE";

/// Constant 'MODE_APM_PLANE_QHOVER'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_QHOVER = "QHOVER";

/// Constant 'MODE_APM_PLANE_QLOITER'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_QLOITER = "QLOITER";

/// Constant 'MODE_APM_PLANE_QLAND'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_QLAND = "QLAND";

/// Constant 'MODE_APM_PLANE_QRTL'.
static const char * const mavros_msgs__msg__State__MODE_APM_PLANE_QRTL = "QRTL";

/// Constant 'MODE_APM_COPTER_STABILIZE'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_STABILIZE = "STABILIZE";

/// Constant 'MODE_APM_COPTER_ACRO'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_ACRO = "ACRO";

/// Constant 'MODE_APM_COPTER_ALT_HOLD'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_ALT_HOLD = "ALT_HOLD";

/// Constant 'MODE_APM_COPTER_AUTO'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_AUTO = "AUTO";

/// Constant 'MODE_APM_COPTER_GUIDED'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_GUIDED = "GUIDED";

/// Constant 'MODE_APM_COPTER_LOITER'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_LOITER = "LOITER";

/// Constant 'MODE_APM_COPTER_RTL'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_RTL = "RTL";

/// Constant 'MODE_APM_COPTER_CIRCLE'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_CIRCLE = "CIRCLE";

/// Constant 'MODE_APM_COPTER_POSITION'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_POSITION = "POSITION";

/// Constant 'MODE_APM_COPTER_LAND'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_LAND = "LAND";

/// Constant 'MODE_APM_COPTER_OF_LOITER'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_OF_LOITER = "OF_LOITER";

/// Constant 'MODE_APM_COPTER_DRIFT'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_DRIFT = "DRIFT";

/// Constant 'MODE_APM_COPTER_SPORT'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_SPORT = "SPORT";

/// Constant 'MODE_APM_COPTER_FLIP'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_FLIP = "FLIP";

/// Constant 'MODE_APM_COPTER_AUTOTUNE'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_AUTOTUNE = "AUTOTUNE";

/// Constant 'MODE_APM_COPTER_POSHOLD'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_POSHOLD = "POSHOLD";

/// Constant 'MODE_APM_COPTER_BRAKE'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_BRAKE = "BRAKE";

/// Constant 'MODE_APM_COPTER_THROW'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_THROW = "THROW";

/// Constant 'MODE_APM_COPTER_AVOID_ADSB'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_AVOID_ADSB = "AVOID_ADSB";

/// Constant 'MODE_APM_COPTER_GUIDED_NOGPS'.
static const char * const mavros_msgs__msg__State__MODE_APM_COPTER_GUIDED_NOGPS = "GUIDED_NOGPS";

/// Constant 'MODE_APM_ROVER_MANUAL'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_MANUAL = "MANUAL";

/// Constant 'MODE_APM_ROVER_LEARNING'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_LEARNING = "LEARNING";

/// Constant 'MODE_APM_ROVER_STEERING'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_STEERING = "STEERING";

/// Constant 'MODE_APM_ROVER_HOLD'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_HOLD = "HOLD";

/// Constant 'MODE_APM_ROVER_AUTO'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_AUTO = "AUTO";

/// Constant 'MODE_APM_ROVER_RTL'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_RTL = "RTL";

/// Constant 'MODE_APM_ROVER_GUIDED'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_GUIDED = "GUIDED";

/// Constant 'MODE_APM_ROVER_INITIALISING'.
static const char * const mavros_msgs__msg__State__MODE_APM_ROVER_INITIALISING = "INITIALISING";

/// Constant 'MODE_PX4_MANUAL'.
static const char * const mavros_msgs__msg__State__MODE_PX4_MANUAL = "MANUAL";

/// Constant 'MODE_PX4_ACRO'.
static const char * const mavros_msgs__msg__State__MODE_PX4_ACRO = "ACRO";

/// Constant 'MODE_PX4_ALTITUDE'.
static const char * const mavros_msgs__msg__State__MODE_PX4_ALTITUDE = "ALTCTL";

/// Constant 'MODE_PX4_POSITION'.
static const char * const mavros_msgs__msg__State__MODE_PX4_POSITION = "POSCTL";

/// Constant 'MODE_PX4_OFFBOARD'.
static const char * const mavros_msgs__msg__State__MODE_PX4_OFFBOARD = "OFFBOARD";

/// Constant 'MODE_PX4_STABILIZED'.
static const char * const mavros_msgs__msg__State__MODE_PX4_STABILIZED = "STABILIZED";

/// Constant 'MODE_PX4_RATTITUDE'.
static const char * const mavros_msgs__msg__State__MODE_PX4_RATTITUDE = "RATTITUDE";

/// Constant 'MODE_PX4_MISSION'.
static const char * const mavros_msgs__msg__State__MODE_PX4_MISSION = "AUTO.MISSION";

/// Constant 'MODE_PX4_LOITER'.
static const char * const mavros_msgs__msg__State__MODE_PX4_LOITER = "AUTO.LOITER";

/// Constant 'MODE_PX4_RTL'.
static const char * const mavros_msgs__msg__State__MODE_PX4_RTL = "AUTO.RTL";

/// Constant 'MODE_PX4_LAND'.
static const char * const mavros_msgs__msg__State__MODE_PX4_LAND = "AUTO.LAND";

/// Constant 'MODE_PX4_RTGS'.
static const char * const mavros_msgs__msg__State__MODE_PX4_RTGS = "AUTO.RTGS";

/// Constant 'MODE_PX4_READY'.
static const char * const mavros_msgs__msg__State__MODE_PX4_READY = "AUTO.READY";

/// Constant 'MODE_PX4_TAKEOFF'.
static const char * const mavros_msgs__msg__State__MODE_PX4_TAKEOFF = "AUTO.TAKEOFF";

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/State in the package mavros_msgs.
/**
  * Current autopilot state
  *
  * Known modes listed here:
  * http://wiki.ros.org/mavros/CustomModes
  *
  * For system_status values
  * see https://mavlink.io/en/messages/common.html#MAV_STATE
 */
typedef struct mavros_msgs__msg__State
{
  std_msgs__msg__Header header;
  bool connected;
  bool armed;
  bool guided;
  bool manual_input;
  rosidl_runtime_c__String mode;
  uint8_t system_status;
} mavros_msgs__msg__State;

// Struct for a sequence of mavros_msgs__msg__State.
typedef struct mavros_msgs__msg__State__Sequence
{
  mavros_msgs__msg__State * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__State__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__STATE__STRUCT_H_
