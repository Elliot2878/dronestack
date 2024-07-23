// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/CommandCode.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__COMMAND_CODE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__COMMAND_CODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'AIRFRAME_CONFIGURATION'.
/**
  * [[[cog:
  * import mavros_cog
  * mavros_cog.idl_decl_enum_mav_cmd()
  * ]]]
  * MAV_CMD_AIRFRAME
 */
enum
{
  mavros_msgs__msg__CommandCode__AIRFRAME_CONFIGURATION = 2520
};

/// Constant 'ARM_AUTHORIZATION_REQUEST'.
/**
  * MAV_CMD_ARM
  * Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
 */
enum
{
  mavros_msgs__msg__CommandCode__ARM_AUTHORIZATION_REQUEST = 3001
};

/// Constant 'CAMERA_TRACK_POINT'.
/**
  * MAV_CMD_CAMERA
  * If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking.
 */
enum
{
  mavros_msgs__msg__CommandCode__CAMERA_TRACK_POINT = 2004
};

/// Constant 'CAMERA_TRACK_RECTANGLE'.
/**
  * If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking.
 */
enum
{
  mavros_msgs__msg__CommandCode__CAMERA_TRACK_RECTANGLE = 2005
};

/// Constant 'CAMERA_STOP_TRACKING'.
/**
  * Stops ongoing tracking.
 */
enum
{
  mavros_msgs__msg__CommandCode__CAMERA_STOP_TRACKING = 2010
};

/// Constant 'CAN_FORWARD'.
/**
  * MAV_CMD_CAN
  * Request forwarding of CAN packets from the given CAN bus to this interface. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages
 */
enum
{
  mavros_msgs__msg__CommandCode__CAN_FORWARD = 32000
};

/// Constant 'COMPONENT_ARM_DISARM'.
/**
  * MAV_CMD_COMPONENT
  * Arms / Disarms a component
 */
enum
{
  mavros_msgs__msg__CommandCode__COMPONENT_ARM_DISARM = 400
};

/// Constant 'CONDITION_DELAY'.
/**
  * MAV_CMD_CONDITION
  * Delay mission state machine.
 */
enum
{
  mavros_msgs__msg__CommandCode__CONDITION_DELAY = 112
};

/// Constant 'CONDITION_CHANGE_ALT'.
/**
  * Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached.
 */
enum
{
  mavros_msgs__msg__CommandCode__CONDITION_CHANGE_ALT = 113
};

/// Constant 'CONDITION_DISTANCE'.
/**
  * Delay mission state machine until within desired distance of next NAV point.
 */
enum
{
  mavros_msgs__msg__CommandCode__CONDITION_DISTANCE = 114
};

/// Constant 'CONDITION_YAW'.
/**
  * Reach a certain target angle.
 */
enum
{
  mavros_msgs__msg__CommandCode__CONDITION_YAW = 115
};

/// Constant 'CONDITION_LAST'.
/**
  * NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
 */
enum
{
  mavros_msgs__msg__CommandCode__CONDITION_LAST = 159
};

/// Constant 'CONTROL_HIGH_LATENCY'.
/**
  * MAV_CMD_CONTROL
  * Request to start/stop transmitting over the high latency telemetry
 */
enum
{
  mavros_msgs__msg__CommandCode__CONTROL_HIGH_LATENCY = 2600
};

/// Constant 'DO_FOLLOW'.
/**
  * MAV_CMD_DO
  * Begin following a target
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_FOLLOW = 32
};

/// Constant 'DO_FOLLOW_REPOSITION'.
/**
  * Reposition the MAV after a follow target command has been sent
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_FOLLOW_REPOSITION = 33
};

/// Constant 'DO_SET_MODE'.
/**
  * Set system mode.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_MODE = 176
};

/// Constant 'DO_JUMP'.
/**
  * Jump to the desired command in the mission list.  Repeat this action only the specified number of times
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_JUMP = 177
};

/// Constant 'DO_CHANGE_SPEED'.
/**
  * Change speed and/or throttle set points
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_CHANGE_SPEED = 178
};

/// Constant 'DO_SET_HOME'.
/**
  * Changes the home location either to the current location or a specified location.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_HOME = 179
};

/// Constant 'DO_SET_PARAMETER'.
/**
  * Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_PARAMETER = 180
};

/// Constant 'DO_SET_RELAY'.
/**
  * Set a relay to a condition.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_RELAY = 181
};

/// Constant 'DO_REPEAT_RELAY'.
/**
  * Cycle a relay on and off for a desired number of cycles with a desired period.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_REPEAT_RELAY = 182
};

/// Constant 'DO_SET_SERVO'.
/**
  * Set a servo to a desired PWM value.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_SERVO = 183
};

/// Constant 'DO_REPEAT_SERVO'.
/**
  * Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_REPEAT_SERVO = 184
};

/// Constant 'DO_FLIGHTTERMINATION'.
/**
  * Terminate flight immediately
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_FLIGHTTERMINATION = 185
};

/// Constant 'DO_CHANGE_ALTITUDE'.
/**
  * Change altitude set point.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_CHANGE_ALTITUDE = 186
};

/// Constant 'DO_LAND_START'.
/**
  * Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude/Altitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_LAND_START = 189
};

/// Constant 'DO_RALLY_LAND'.
/**
  * Mission command to perform a landing from a rally point.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_RALLY_LAND = 190
};

/// Constant 'DO_GO_AROUND'.
/**
  * Mission command to safely abort an autonomous landing.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_GO_AROUND = 191
};

/// Constant 'DO_REPOSITION'.
/**
  * Reposition the vehicle to a specific WGS84 global position.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_REPOSITION = 192
};

/// Constant 'DO_PAUSE_CONTINUE'.
/**
  * If in a GPS controlled position mode, hold the current position or continue.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_PAUSE_CONTINUE = 193
};

/// Constant 'DO_SET_REVERSE'.
/**
  * Set moving direction to forward or reverse.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_REVERSE = 194
};

/// Constant 'DO_SET_ROI_LOCATION'.
/**
  * Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_ROI_LOCATION = 195
};

/// Constant 'DO_SET_ROI_WPNEXT_OFFSET'.
/**
  * Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_ROI_WPNEXT_OFFSET = 196
};

/// Constant 'DO_SET_ROI_NONE'.
/**
  * Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_ROI_NONE = 197
};

/// Constant 'DO_SET_ROI_SYSID'.
/**
  * Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_ROI_SYSID = 198
};

/// Constant 'DO_CONTROL_VIDEO'.
/**
  * Control onboard camera system.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_CONTROL_VIDEO = 200
};

/// Constant 'DO_SET_ROI'.
/**
  * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_ROI = 201
};

/// Constant 'DO_DIGICAM_CONFIGURE'.
/**
  * Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_DIGICAM_CONFIGURE = 202
};

/// Constant 'DO_DIGICAM_CONTROL'.
/**
  * Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_DIGICAM_CONTROL = 203
};

/// Constant 'DO_MOUNT_CONFIGURE'.
/**
  * Mission command to configure a camera or antenna mount
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_MOUNT_CONFIGURE = 204
};

/// Constant 'DO_MOUNT_CONTROL'.
/**
  * Mission command to control a camera or antenna mount
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_MOUNT_CONTROL = 205
};

/// Constant 'DO_SET_CAM_TRIGG_DIST'.
/**
  * Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_CAM_TRIGG_DIST = 206
};

/// Constant 'DO_FENCE_ENABLE'.
/**
  * Mission command to enable the geofence
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_FENCE_ENABLE = 207
};

/// Constant 'DO_PARACHUTE'.
/**
  * Mission item/command to release a parachute or enable/disable auto release.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_PARACHUTE = 208
};

/// Constant 'DO_MOTOR_TEST'.
/**
  * Mission command to perform motor test.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_MOTOR_TEST = 209
};

/// Constant 'DO_INVERTED_FLIGHT'.
/**
  * Change to/from inverted flight.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_INVERTED_FLIGHT = 210
};

/// Constant 'DO_GRIPPER'.
/**
  * Mission command to operate a gripper.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_GRIPPER = 211
};

/// Constant 'DO_AUTOTUNE_ENABLE'.
/**
  * Enable/disable autotune.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_AUTOTUNE_ENABLE = 212
};

/// Constant 'DO_SET_CAM_TRIGG_INTERVAL'.
/**
  * Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_CAM_TRIGG_INTERVAL = 214
};

/// Constant 'DO_MOUNT_CONTROL_QUAT'.
/**
  * Mission command to control a camera or antenna mount, using a quaternion as reference.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_MOUNT_CONTROL_QUAT = 220
};

/// Constant 'DO_GUIDED_MASTER'.
/**
  * set id of master controller
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_GUIDED_MASTER = 221
};

/// Constant 'DO_GUIDED_LIMITS'.
/**
  * Set limits for external control
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_GUIDED_LIMITS = 222
};

/// Constant 'DO_ENGINE_CONTROL'.
/**
  * Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_ENGINE_CONTROL = 223
};

/// Constant 'DO_SET_MISSION_CURRENT'.
/**
  * Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_SET_MISSION_CURRENT = 224
};

/// Constant 'DO_LAST'.
/**
  * NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_LAST = 240
};

/// Constant 'DO_JUMP_TAG'.
/**
  * Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_JUMP_TAG = 601
};

/// Constant 'DO_GIMBAL_MANAGER_PITCHYAW'.
/**
  * Set gimbal manager pitch/yaw setpoints (low rate command). It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: only the gimbal manager will react to this command - it will be ignored by a gimbal device. Use GIMBAL_MANAGER_SET_PITCHYAW if you need to stream pitch/yaw setpoints at higher rate.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_GIMBAL_MANAGER_PITCHYAW = 1000
};

/// Constant 'DO_GIMBAL_MANAGER_CONFIGURE'.
/**
  * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_GIMBAL_MANAGER_CONFIGURE = 1001
};

/// Constant 'DO_TRIGGER_CONTROL'.
/**
  * Enable or disable on-board camera triggering system.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_TRIGGER_CONTROL = 2003
};

/// Constant 'DO_VTOL_TRANSITION'.
/**
  * Request VTOL transition
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_VTOL_TRANSITION = 3000
};

/// Constant 'DO_ADSB_OUT_IDENT'.
/**
  * Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_ADSB_OUT_IDENT = 10001
};

/// Constant 'DO_WINCH'.
/**
  * Command to operate winch.
 */
enum
{
  mavros_msgs__msg__CommandCode__DO_WINCH = 42600
};

/// Constant 'FIXED_MAG_CAL_YAW'.
/**
  * MAV_CMD_FIXED
  * Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.
 */
enum
{
  mavros_msgs__msg__CommandCode__FIXED_MAG_CAL_YAW = 42006
};

/// Constant 'GET_HOME_POSITION'.
/**
  * MAV_CMD_GET
  * Request the home position from the vehicle.
 */
enum
{
  mavros_msgs__msg__CommandCode__GET_HOME_POSITION = 410
};

/// Constant 'GET_MESSAGE_INTERVAL'.
/**
  * Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.
 */
enum
{
  mavros_msgs__msg__CommandCode__GET_MESSAGE_INTERVAL = 510
};

/// Constant 'IMAGE_START_CAPTURE'.
/**
  * MAV_CMD_IMAGE
  * Start image capture sequence. CAMERA_IMAGE_CAPTURED must be emitted after each capture. Param1 (id) may be used to specify the target camera: 0: all cameras, 1 to 6: autopilot-connected cameras, 7-255: MAVLink camera component ID. It is needed in order to target specific cameras connected to the autopilot, or specific sensors in a multi-sensor camera (neither of which have a distinct MAVLink component ID). It is also needed to specify the target camera in missions. When used in a mission, an autopilot should execute the MAV_CMD for a specified local camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink camera (param1 = 7 - 255), setting the command's target_component as the param1 value (and setting param1 in the command to zero). If the param1 is 0 the autopilot should do both. When sent in a command the target MAVLink address is set using target_component. If addressed specifically to an autopilot: param1 should be used in the same way as it is for missions (though command should NACK with MAV_RESULT_DENIED if a specified local camera does not exist). If addressed to a MAVLink camera, param 1 can be used to address all cameras (0), or to separately address 1 to 7 individual sensors. Other values should be NACKed with MAV_RESULT_DENIED. If the command is broadcast (target_component is 0) then param 1 should be set to 0 (any other value should be NACKED with MAV_RESULT_DENIED). An autopilot would trigger any local cameras and forward the command to all channels.
 */
enum
{
  mavros_msgs__msg__CommandCode__IMAGE_START_CAPTURE = 2000
};

/// Constant 'IMAGE_STOP_CAPTURE'.
/**
  * Stop image capture sequence. Param1 (id) may be used to specify the target camera: 0: all cameras, 1 to 6: autopilot-connected cameras, 7-255: MAVLink camera component ID. It is needed in order to target specific cameras connected to the autopilot, or specific sensors in a multi-sensor camera (neither of which have a distinct MAVLink component ID). It is also needed to specify the target camera in missions. When used in a mission, an autopilot should execute the MAV_CMD for a specified local camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink camera (param1 = 7 - 255), setting the command's target_component as the param1 value (and setting param1 in the command to zero). If the param1 is 0 the autopilot should do both. When sent in a command the target MAVLink address is set using target_component. If addressed specifically to an autopilot: param1 should be used in the same way as it is for missions (though command should NACK with MAV_RESULT_DENIED if a specified local camera does not exist). If addressed to a MAVLink camera, param1 can be used to address all cameras (0), or to separately address 1 to 7 individual sensors. Other values should be NACKed with MAV_RESULT_DENIED. If the command is broadcast (target_component is 0) then param 1 should be set to 0 (any other value should be NACKED with MAV_RESULT_DENIED). An autopilot would trigger any local cameras and forward the command to all channels.
 */
enum
{
  mavros_msgs__msg__CommandCode__IMAGE_STOP_CAPTURE = 2001
};

/// Constant 'JUMP_TAG'.
/**
  * MAV_CMD_JUMP
  * Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
 */
enum
{
  mavros_msgs__msg__CommandCode__JUMP_TAG = 600
};

/// Constant 'LOGGING_START'.
/**
  * MAV_CMD_LOGGING
  * Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
 */
enum
{
  mavros_msgs__msg__CommandCode__LOGGING_START = 2510
};

/// Constant 'LOGGING_STOP'.
/**
  * Request to stop streaming log data over MAVLink
 */
enum
{
  mavros_msgs__msg__CommandCode__LOGGING_STOP = 2511
};

/// Constant 'MISSION_START'.
/**
  * MAV_CMD_MISSION
  * start running a mission
 */
enum
{
  mavros_msgs__msg__CommandCode__MISSION_START = 300
};

/// Constant 'NAV_WAYPOINT'.
/**
  * MAV_CMD_NAV
  * Navigate to waypoint.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_WAYPOINT = 16
};

/// Constant 'NAV_LOITER_UNLIM'.
/**
  * Loiter around this waypoint an unlimited amount of time
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LOITER_UNLIM = 17
};

/// Constant 'NAV_LOITER_TURNS'.
/**
  * Loiter around this waypoint for X turns
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LOITER_TURNS = 18
};

/// Constant 'NAV_LOITER_TIME'.
/**
  * Loiter around this waypoint for X seconds
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LOITER_TIME = 19
};

/// Constant 'NAV_RETURN_TO_LAUNCH'.
/**
  * Return to launch location
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_RETURN_TO_LAUNCH = 20
};

/// Constant 'NAV_LAND'.
/**
  * Land at location.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LAND = 21
};

/// Constant 'NAV_TAKEOFF'.
/**
  * Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_TAKEOFF = 22
};

/// Constant 'NAV_LAND_LOCAL'.
/**
  * Land at local position (local frame only)
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LAND_LOCAL = 23
};

/// Constant 'NAV_TAKEOFF_LOCAL'.
/**
  * Takeoff from local position (local frame only)
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_TAKEOFF_LOCAL = 24
};

/// Constant 'NAV_FOLLOW'.
/**
  * Vehicle following, i.e. this waypoint represents the position of a moving vehicle
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_FOLLOW = 25
};

/// Constant 'NAV_CONTINUE_AND_CHANGE_ALT'.
/**
  * Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_CONTINUE_AND_CHANGE_ALT = 30
};

/// Constant 'NAV_LOITER_TO_ALT'.
/**
  * Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LOITER_TO_ALT = 31
};

/// Constant 'NAV_ROI'.
/**
  * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_ROI = 80
};

/// Constant 'NAV_PATHPLANNING'.
/**
  * Control autonomous path planning on the MAV.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_PATHPLANNING = 81
};

/// Constant 'NAV_SPLINE_WAYPOINT'.
/**
  * Navigate to waypoint using a spline path.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_SPLINE_WAYPOINT = 82
};

/// Constant 'NAV_VTOL_TAKEOFF'.
/**
  * Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_VTOL_TAKEOFF = 84
};

/// Constant 'NAV_VTOL_LAND'.
/**
  * Land using VTOL mode
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_VTOL_LAND = 85
};

/// Constant 'NAV_GUIDED_ENABLE'.
/**
  * hand control over to an external controller
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_GUIDED_ENABLE = 92
};

/// Constant 'NAV_DELAY'.
/**
  * Delay the next navigation command a number of seconds or until a specified time
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_DELAY = 93
};

/// Constant 'NAV_PAYLOAD_PLACE'.
/**
  * Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_PAYLOAD_PLACE = 94
};

/// Constant 'NAV_LAST'.
/**
  * NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_LAST = 95
};

/// Constant 'NAV_SET_YAW_SPEED'.
/**
  * Sets a desired vehicle turn angle and speed change.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_SET_YAW_SPEED = 213
};

/// Constant 'NAV_FENCE_RETURN_POINT'.
/**
  * Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_FENCE_RETURN_POINT = 5000
};

/// Constant 'NAV_FENCE_POLYGON_VERTEX_INCLUSION'.
/**
  * Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
};

/// Constant 'NAV_FENCE_POLYGON_VERTEX_EXCLUSION'.
/**
  * Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
};

/// Constant 'NAV_FENCE_CIRCLE_INCLUSION'.
/**
  * Circular fence area. The vehicle must stay inside this area.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_FENCE_CIRCLE_INCLUSION = 5003
};

/// Constant 'NAV_FENCE_CIRCLE_EXCLUSION'.
/**
  * Circular fence area. The vehicle must stay outside this area.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_FENCE_CIRCLE_EXCLUSION = 5004
};

/// Constant 'NAV_RALLY_POINT'.
/**
  * Rally point. You can have multiple rally points defined.
 */
enum
{
  mavros_msgs__msg__CommandCode__NAV_RALLY_POINT = 5100
};

/// Constant 'OBLIQUE_SURVEY'.
/**
  * MAV_CMD_OBLIQUE
  * Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera.
 */
enum
{
  mavros_msgs__msg__CommandCode__OBLIQUE_SURVEY = 260
};

/// Constant 'OVERRIDE_GOTO'.
/**
  * MAV_CMD_OVERRIDE
  * Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.
 */
enum
{
  mavros_msgs__msg__CommandCode__OVERRIDE_GOTO = 252
};

/// Constant 'PANORAMA_CREATE'.
/**
  * MAV_CMD_PANORAMA
  * Create a panorama at the current position
 */
enum
{
  mavros_msgs__msg__CommandCode__PANORAMA_CREATE = 2800
};

/// Constant 'PAYLOAD_PREPARE_DEPLOY'.
/**
  * MAV_CMD_PAYLOAD
  * Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
 */
enum
{
  mavros_msgs__msg__CommandCode__PAYLOAD_PREPARE_DEPLOY = 30001
};

/// Constant 'PAYLOAD_CONTROL_DEPLOY'.
/**
  * Control the payload deployment.
 */
enum
{
  mavros_msgs__msg__CommandCode__PAYLOAD_CONTROL_DEPLOY = 30002
};

/// Constant 'PREFLIGHT_CALIBRATION'.
/**
  * MAV_CMD_PREFLIGHT
  * Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
 */
enum
{
  mavros_msgs__msg__CommandCode__PREFLIGHT_CALIBRATION = 241
};

/// Constant 'PREFLIGHT_SET_SENSOR_OFFSETS'.
/**
  * Set sensor offsets. This command will be only accepted if in pre-flight mode.
 */
enum
{
  mavros_msgs__msg__CommandCode__PREFLIGHT_SET_SENSOR_OFFSETS = 242
};

/// Constant 'PREFLIGHT_UAVCAN'.
/**
  * Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
 */
enum
{
  mavros_msgs__msg__CommandCode__PREFLIGHT_UAVCAN = 243
};

/// Constant 'PREFLIGHT_STORAGE'.
/**
  * Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
 */
enum
{
  mavros_msgs__msg__CommandCode__PREFLIGHT_STORAGE = 245
};

/// Constant 'PREFLIGHT_REBOOT_SHUTDOWN'.
/**
  * Request the reboot or shutdown of system components.
 */
enum
{
  mavros_msgs__msg__CommandCode__PREFLIGHT_REBOOT_SHUTDOWN = 246
};

/// Constant 'REQUEST_MESSAGE'.
/**
  * MAV_CMD_REQUEST
  * Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_MESSAGE = 512
};

/// Constant 'REQUEST_PROTOCOL_VERSION'.
/**
  * Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_PROTOCOL_VERSION = 519
};

/// Constant 'REQUEST_AUTOPILOT_CAPABILITIES'.
/**
  * Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_AUTOPILOT_CAPABILITIES = 520
};

/// Constant 'REQUEST_CAMERA_INFORMATION'.
/**
  * Request camera information (CAMERA_INFORMATION).
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_CAMERA_INFORMATION = 521
};

/// Constant 'REQUEST_CAMERA_SETTINGS'.
/**
  * Request camera settings (CAMERA_SETTINGS).
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_CAMERA_SETTINGS = 522
};

/// Constant 'REQUEST_STORAGE_INFORMATION'.
/**
  * Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_STORAGE_INFORMATION = 525
};

/// Constant 'REQUEST_CAMERA_CAPTURE_STATUS'.
/**
  * Request camera capture status (CAMERA_CAPTURE_STATUS)
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_CAMERA_CAPTURE_STATUS = 527
};

/// Constant 'REQUEST_FLIGHT_INFORMATION'.
/**
  * Request flight information (FLIGHT_INFORMATION)
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_FLIGHT_INFORMATION = 528
};

/// Constant 'REQUEST_VIDEO_STREAM_INFORMATION'.
/**
  * Request video stream information (VIDEO_STREAM_INFORMATION)
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_VIDEO_STREAM_INFORMATION = 2504
};

/// Constant 'REQUEST_VIDEO_STREAM_STATUS'.
/**
  * Request video stream status (VIDEO_STREAM_STATUS)
 */
enum
{
  mavros_msgs__msg__CommandCode__REQUEST_VIDEO_STREAM_STATUS = 2505
};

/// Constant 'RESET_CAMERA_SETTINGS'.
/**
  * MAV_CMD_RESET
  * Reset all camera settings to Factory Default
 */
enum
{
  mavros_msgs__msg__CommandCode__RESET_CAMERA_SETTINGS = 529
};

/// Constant 'RUN_PREARM_CHECKS'.
/**
  * MAV_CMD_RUN
  * Instructs system to run pre-arm checks.  This command should return MAV_RESULT_TEMPORARILY_REJECTED in the case the system is armed, otherwse MAV_RESULT_ACCEPTED.  Note that the return value from executing this command does not indicate whether the vehicle is armable or not, just whether the system has successfully run/is currently running the checks.  The result of the checks is reflected in the SYS_STATUS message.
 */
enum
{
  mavros_msgs__msg__CommandCode__RUN_PREARM_CHECKS = 401
};

/// Constant 'SET_MESSAGE_INTERVAL'.
/**
  * MAV_CMD_SET
  * Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.
 */
enum
{
  mavros_msgs__msg__CommandCode__SET_MESSAGE_INTERVAL = 511
};

/// Constant 'SET_CAMERA_MODE'.
/**
  * Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.
 */
enum
{
  mavros_msgs__msg__CommandCode__SET_CAMERA_MODE = 530
};

/// Constant 'SET_CAMERA_ZOOM'.
/**
  * Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
 */
enum
{
  mavros_msgs__msg__CommandCode__SET_CAMERA_ZOOM = 531
};

/// Constant 'SET_CAMERA_FOCUS'.
/**
  * Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
 */
enum
{
  mavros_msgs__msg__CommandCode__SET_CAMERA_FOCUS = 532
};

/// Constant 'SET_GUIDED_SUBMODE_STANDARD'.
/**
  * This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
 */
enum
{
  mavros_msgs__msg__CommandCode__SET_GUIDED_SUBMODE_STANDARD = 4000
};

/// Constant 'SET_GUIDED_SUBMODE_CIRCLE'.
/**
  * This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
 */
enum
{
  mavros_msgs__msg__CommandCode__SET_GUIDED_SUBMODE_CIRCLE = 4001
};

/// Constant 'START_RX_PAIR'.
/**
  * MAV_CMD_START
  * Starts receiver pairing.
 */
enum
{
  mavros_msgs__msg__CommandCode__START_RX_PAIR = 500
};

/// Constant 'STORAGE_FORMAT'.
/**
  * MAV_CMD_STORAGE
  * Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
 */
enum
{
  mavros_msgs__msg__CommandCode__STORAGE_FORMAT = 526
};

/// Constant 'UAVCAN_GET_NODE_INFO'.
/**
  * MAV_CMD_UAVCAN
  * Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
 */
enum
{
  mavros_msgs__msg__CommandCode__UAVCAN_GET_NODE_INFO = 5200
};

/// Constant 'VIDEO_START_CAPTURE'.
/**
  * MAV_CMD_VIDEO
  * Starts video capture (recording).
 */
enum
{
  mavros_msgs__msg__CommandCode__VIDEO_START_CAPTURE = 2500
};

/// Constant 'VIDEO_STOP_CAPTURE'.
/**
  * Stop the current video capture (recording).
 */
enum
{
  mavros_msgs__msg__CommandCode__VIDEO_STOP_CAPTURE = 2501
};

/// Constant 'VIDEO_START_STREAMING'.
/**
  * Start video streaming
 */
enum
{
  mavros_msgs__msg__CommandCode__VIDEO_START_STREAMING = 2502
};

/// Constant 'VIDEO_STOP_STREAMING'.
/**
  * Stop the given video stream
 */
enum
{
  mavros_msgs__msg__CommandCode__VIDEO_STOP_STREAMING = 2503
};

/// Struct defined in msg/CommandCode in the package mavros_msgs.
/**
  * MAV_CMD command codes.
  * Actual meaning and params you may find in MAVLink documentation
  * https://mavlink.io/en/messages/common.html#MAV_CMD
 */
typedef struct mavros_msgs__msg__CommandCode
{
  uint8_t structure_needs_at_least_one_member;
} mavros_msgs__msg__CommandCode;

// Struct for a sequence of mavros_msgs__msg__CommandCode.
typedef struct mavros_msgs__msg__CommandCode__Sequence
{
  mavros_msgs__msg__CommandCode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__CommandCode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__COMMAND_CODE__STRUCT_H_
