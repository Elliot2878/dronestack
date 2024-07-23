# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/CommandCode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CommandCode(type):
    """Metaclass of message 'CommandCode'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'AIRFRAME_CONFIGURATION': 2520,
        'ARM_AUTHORIZATION_REQUEST': 3001,
        'CAMERA_TRACK_POINT': 2004,
        'CAMERA_TRACK_RECTANGLE': 2005,
        'CAMERA_STOP_TRACKING': 2010,
        'CAN_FORWARD': 32000,
        'COMPONENT_ARM_DISARM': 400,
        'CONDITION_DELAY': 112,
        'CONDITION_CHANGE_ALT': 113,
        'CONDITION_DISTANCE': 114,
        'CONDITION_YAW': 115,
        'CONDITION_LAST': 159,
        'CONTROL_HIGH_LATENCY': 2600,
        'DO_FOLLOW': 32,
        'DO_FOLLOW_REPOSITION': 33,
        'DO_SET_MODE': 176,
        'DO_JUMP': 177,
        'DO_CHANGE_SPEED': 178,
        'DO_SET_HOME': 179,
        'DO_SET_PARAMETER': 180,
        'DO_SET_RELAY': 181,
        'DO_REPEAT_RELAY': 182,
        'DO_SET_SERVO': 183,
        'DO_REPEAT_SERVO': 184,
        'DO_FLIGHTTERMINATION': 185,
        'DO_CHANGE_ALTITUDE': 186,
        'DO_LAND_START': 189,
        'DO_RALLY_LAND': 190,
        'DO_GO_AROUND': 191,
        'DO_REPOSITION': 192,
        'DO_PAUSE_CONTINUE': 193,
        'DO_SET_REVERSE': 194,
        'DO_SET_ROI_LOCATION': 195,
        'DO_SET_ROI_WPNEXT_OFFSET': 196,
        'DO_SET_ROI_NONE': 197,
        'DO_SET_ROI_SYSID': 198,
        'DO_CONTROL_VIDEO': 200,
        'DO_SET_ROI': 201,
        'DO_DIGICAM_CONFIGURE': 202,
        'DO_DIGICAM_CONTROL': 203,
        'DO_MOUNT_CONFIGURE': 204,
        'DO_MOUNT_CONTROL': 205,
        'DO_SET_CAM_TRIGG_DIST': 206,
        'DO_FENCE_ENABLE': 207,
        'DO_PARACHUTE': 208,
        'DO_MOTOR_TEST': 209,
        'DO_INVERTED_FLIGHT': 210,
        'DO_GRIPPER': 211,
        'DO_AUTOTUNE_ENABLE': 212,
        'DO_SET_CAM_TRIGG_INTERVAL': 214,
        'DO_MOUNT_CONTROL_QUAT': 220,
        'DO_GUIDED_MASTER': 221,
        'DO_GUIDED_LIMITS': 222,
        'DO_ENGINE_CONTROL': 223,
        'DO_SET_MISSION_CURRENT': 224,
        'DO_LAST': 240,
        'DO_JUMP_TAG': 601,
        'DO_GIMBAL_MANAGER_PITCHYAW': 1000,
        'DO_GIMBAL_MANAGER_CONFIGURE': 1001,
        'DO_TRIGGER_CONTROL': 2003,
        'DO_VTOL_TRANSITION': 3000,
        'DO_ADSB_OUT_IDENT': 10001,
        'DO_WINCH': 42600,
        'FIXED_MAG_CAL_YAW': 42006,
        'GET_HOME_POSITION': 410,
        'GET_MESSAGE_INTERVAL': 510,
        'IMAGE_START_CAPTURE': 2000,
        'IMAGE_STOP_CAPTURE': 2001,
        'JUMP_TAG': 600,
        'LOGGING_START': 2510,
        'LOGGING_STOP': 2511,
        'MISSION_START': 300,
        'NAV_WAYPOINT': 16,
        'NAV_LOITER_UNLIM': 17,
        'NAV_LOITER_TURNS': 18,
        'NAV_LOITER_TIME': 19,
        'NAV_RETURN_TO_LAUNCH': 20,
        'NAV_LAND': 21,
        'NAV_TAKEOFF': 22,
        'NAV_LAND_LOCAL': 23,
        'NAV_TAKEOFF_LOCAL': 24,
        'NAV_FOLLOW': 25,
        'NAV_CONTINUE_AND_CHANGE_ALT': 30,
        'NAV_LOITER_TO_ALT': 31,
        'NAV_ROI': 80,
        'NAV_PATHPLANNING': 81,
        'NAV_SPLINE_WAYPOINT': 82,
        'NAV_VTOL_TAKEOFF': 84,
        'NAV_VTOL_LAND': 85,
        'NAV_GUIDED_ENABLE': 92,
        'NAV_DELAY': 93,
        'NAV_PAYLOAD_PLACE': 94,
        'NAV_LAST': 95,
        'NAV_SET_YAW_SPEED': 213,
        'NAV_FENCE_RETURN_POINT': 5000,
        'NAV_FENCE_POLYGON_VERTEX_INCLUSION': 5001,
        'NAV_FENCE_POLYGON_VERTEX_EXCLUSION': 5002,
        'NAV_FENCE_CIRCLE_INCLUSION': 5003,
        'NAV_FENCE_CIRCLE_EXCLUSION': 5004,
        'NAV_RALLY_POINT': 5100,
        'OBLIQUE_SURVEY': 260,
        'OVERRIDE_GOTO': 252,
        'PANORAMA_CREATE': 2800,
        'PAYLOAD_PREPARE_DEPLOY': 30001,
        'PAYLOAD_CONTROL_DEPLOY': 30002,
        'PREFLIGHT_CALIBRATION': 241,
        'PREFLIGHT_SET_SENSOR_OFFSETS': 242,
        'PREFLIGHT_UAVCAN': 243,
        'PREFLIGHT_STORAGE': 245,
        'PREFLIGHT_REBOOT_SHUTDOWN': 246,
        'REQUEST_MESSAGE': 512,
        'REQUEST_PROTOCOL_VERSION': 519,
        'REQUEST_AUTOPILOT_CAPABILITIES': 520,
        'REQUEST_CAMERA_INFORMATION': 521,
        'REQUEST_CAMERA_SETTINGS': 522,
        'REQUEST_STORAGE_INFORMATION': 525,
        'REQUEST_CAMERA_CAPTURE_STATUS': 527,
        'REQUEST_FLIGHT_INFORMATION': 528,
        'REQUEST_VIDEO_STREAM_INFORMATION': 2504,
        'REQUEST_VIDEO_STREAM_STATUS': 2505,
        'RESET_CAMERA_SETTINGS': 529,
        'RUN_PREARM_CHECKS': 401,
        'SET_MESSAGE_INTERVAL': 511,
        'SET_CAMERA_MODE': 530,
        'SET_CAMERA_ZOOM': 531,
        'SET_CAMERA_FOCUS': 532,
        'SET_GUIDED_SUBMODE_STANDARD': 4000,
        'SET_GUIDED_SUBMODE_CIRCLE': 4001,
        'START_RX_PAIR': 500,
        'STORAGE_FORMAT': 526,
        'UAVCAN_GET_NODE_INFO': 5200,
        'VIDEO_START_CAPTURE': 2500,
        'VIDEO_STOP_CAPTURE': 2501,
        'VIDEO_START_STREAMING': 2502,
        'VIDEO_STOP_STREAMING': 2503,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mavros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mavros_msgs.msg.CommandCode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__command_code
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__command_code
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__command_code
            cls._TYPE_SUPPORT = module.type_support_msg__msg__command_code
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__command_code

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'AIRFRAME_CONFIGURATION': cls.__constants['AIRFRAME_CONFIGURATION'],
            'ARM_AUTHORIZATION_REQUEST': cls.__constants['ARM_AUTHORIZATION_REQUEST'],
            'CAMERA_TRACK_POINT': cls.__constants['CAMERA_TRACK_POINT'],
            'CAMERA_TRACK_RECTANGLE': cls.__constants['CAMERA_TRACK_RECTANGLE'],
            'CAMERA_STOP_TRACKING': cls.__constants['CAMERA_STOP_TRACKING'],
            'CAN_FORWARD': cls.__constants['CAN_FORWARD'],
            'COMPONENT_ARM_DISARM': cls.__constants['COMPONENT_ARM_DISARM'],
            'CONDITION_DELAY': cls.__constants['CONDITION_DELAY'],
            'CONDITION_CHANGE_ALT': cls.__constants['CONDITION_CHANGE_ALT'],
            'CONDITION_DISTANCE': cls.__constants['CONDITION_DISTANCE'],
            'CONDITION_YAW': cls.__constants['CONDITION_YAW'],
            'CONDITION_LAST': cls.__constants['CONDITION_LAST'],
            'CONTROL_HIGH_LATENCY': cls.__constants['CONTROL_HIGH_LATENCY'],
            'DO_FOLLOW': cls.__constants['DO_FOLLOW'],
            'DO_FOLLOW_REPOSITION': cls.__constants['DO_FOLLOW_REPOSITION'],
            'DO_SET_MODE': cls.__constants['DO_SET_MODE'],
            'DO_JUMP': cls.__constants['DO_JUMP'],
            'DO_CHANGE_SPEED': cls.__constants['DO_CHANGE_SPEED'],
            'DO_SET_HOME': cls.__constants['DO_SET_HOME'],
            'DO_SET_PARAMETER': cls.__constants['DO_SET_PARAMETER'],
            'DO_SET_RELAY': cls.__constants['DO_SET_RELAY'],
            'DO_REPEAT_RELAY': cls.__constants['DO_REPEAT_RELAY'],
            'DO_SET_SERVO': cls.__constants['DO_SET_SERVO'],
            'DO_REPEAT_SERVO': cls.__constants['DO_REPEAT_SERVO'],
            'DO_FLIGHTTERMINATION': cls.__constants['DO_FLIGHTTERMINATION'],
            'DO_CHANGE_ALTITUDE': cls.__constants['DO_CHANGE_ALTITUDE'],
            'DO_LAND_START': cls.__constants['DO_LAND_START'],
            'DO_RALLY_LAND': cls.__constants['DO_RALLY_LAND'],
            'DO_GO_AROUND': cls.__constants['DO_GO_AROUND'],
            'DO_REPOSITION': cls.__constants['DO_REPOSITION'],
            'DO_PAUSE_CONTINUE': cls.__constants['DO_PAUSE_CONTINUE'],
            'DO_SET_REVERSE': cls.__constants['DO_SET_REVERSE'],
            'DO_SET_ROI_LOCATION': cls.__constants['DO_SET_ROI_LOCATION'],
            'DO_SET_ROI_WPNEXT_OFFSET': cls.__constants['DO_SET_ROI_WPNEXT_OFFSET'],
            'DO_SET_ROI_NONE': cls.__constants['DO_SET_ROI_NONE'],
            'DO_SET_ROI_SYSID': cls.__constants['DO_SET_ROI_SYSID'],
            'DO_CONTROL_VIDEO': cls.__constants['DO_CONTROL_VIDEO'],
            'DO_SET_ROI': cls.__constants['DO_SET_ROI'],
            'DO_DIGICAM_CONFIGURE': cls.__constants['DO_DIGICAM_CONFIGURE'],
            'DO_DIGICAM_CONTROL': cls.__constants['DO_DIGICAM_CONTROL'],
            'DO_MOUNT_CONFIGURE': cls.__constants['DO_MOUNT_CONFIGURE'],
            'DO_MOUNT_CONTROL': cls.__constants['DO_MOUNT_CONTROL'],
            'DO_SET_CAM_TRIGG_DIST': cls.__constants['DO_SET_CAM_TRIGG_DIST'],
            'DO_FENCE_ENABLE': cls.__constants['DO_FENCE_ENABLE'],
            'DO_PARACHUTE': cls.__constants['DO_PARACHUTE'],
            'DO_MOTOR_TEST': cls.__constants['DO_MOTOR_TEST'],
            'DO_INVERTED_FLIGHT': cls.__constants['DO_INVERTED_FLIGHT'],
            'DO_GRIPPER': cls.__constants['DO_GRIPPER'],
            'DO_AUTOTUNE_ENABLE': cls.__constants['DO_AUTOTUNE_ENABLE'],
            'DO_SET_CAM_TRIGG_INTERVAL': cls.__constants['DO_SET_CAM_TRIGG_INTERVAL'],
            'DO_MOUNT_CONTROL_QUAT': cls.__constants['DO_MOUNT_CONTROL_QUAT'],
            'DO_GUIDED_MASTER': cls.__constants['DO_GUIDED_MASTER'],
            'DO_GUIDED_LIMITS': cls.__constants['DO_GUIDED_LIMITS'],
            'DO_ENGINE_CONTROL': cls.__constants['DO_ENGINE_CONTROL'],
            'DO_SET_MISSION_CURRENT': cls.__constants['DO_SET_MISSION_CURRENT'],
            'DO_LAST': cls.__constants['DO_LAST'],
            'DO_JUMP_TAG': cls.__constants['DO_JUMP_TAG'],
            'DO_GIMBAL_MANAGER_PITCHYAW': cls.__constants['DO_GIMBAL_MANAGER_PITCHYAW'],
            'DO_GIMBAL_MANAGER_CONFIGURE': cls.__constants['DO_GIMBAL_MANAGER_CONFIGURE'],
            'DO_TRIGGER_CONTROL': cls.__constants['DO_TRIGGER_CONTROL'],
            'DO_VTOL_TRANSITION': cls.__constants['DO_VTOL_TRANSITION'],
            'DO_ADSB_OUT_IDENT': cls.__constants['DO_ADSB_OUT_IDENT'],
            'DO_WINCH': cls.__constants['DO_WINCH'],
            'FIXED_MAG_CAL_YAW': cls.__constants['FIXED_MAG_CAL_YAW'],
            'GET_HOME_POSITION': cls.__constants['GET_HOME_POSITION'],
            'GET_MESSAGE_INTERVAL': cls.__constants['GET_MESSAGE_INTERVAL'],
            'IMAGE_START_CAPTURE': cls.__constants['IMAGE_START_CAPTURE'],
            'IMAGE_STOP_CAPTURE': cls.__constants['IMAGE_STOP_CAPTURE'],
            'JUMP_TAG': cls.__constants['JUMP_TAG'],
            'LOGGING_START': cls.__constants['LOGGING_START'],
            'LOGGING_STOP': cls.__constants['LOGGING_STOP'],
            'MISSION_START': cls.__constants['MISSION_START'],
            'NAV_WAYPOINT': cls.__constants['NAV_WAYPOINT'],
            'NAV_LOITER_UNLIM': cls.__constants['NAV_LOITER_UNLIM'],
            'NAV_LOITER_TURNS': cls.__constants['NAV_LOITER_TURNS'],
            'NAV_LOITER_TIME': cls.__constants['NAV_LOITER_TIME'],
            'NAV_RETURN_TO_LAUNCH': cls.__constants['NAV_RETURN_TO_LAUNCH'],
            'NAV_LAND': cls.__constants['NAV_LAND'],
            'NAV_TAKEOFF': cls.__constants['NAV_TAKEOFF'],
            'NAV_LAND_LOCAL': cls.__constants['NAV_LAND_LOCAL'],
            'NAV_TAKEOFF_LOCAL': cls.__constants['NAV_TAKEOFF_LOCAL'],
            'NAV_FOLLOW': cls.__constants['NAV_FOLLOW'],
            'NAV_CONTINUE_AND_CHANGE_ALT': cls.__constants['NAV_CONTINUE_AND_CHANGE_ALT'],
            'NAV_LOITER_TO_ALT': cls.__constants['NAV_LOITER_TO_ALT'],
            'NAV_ROI': cls.__constants['NAV_ROI'],
            'NAV_PATHPLANNING': cls.__constants['NAV_PATHPLANNING'],
            'NAV_SPLINE_WAYPOINT': cls.__constants['NAV_SPLINE_WAYPOINT'],
            'NAV_VTOL_TAKEOFF': cls.__constants['NAV_VTOL_TAKEOFF'],
            'NAV_VTOL_LAND': cls.__constants['NAV_VTOL_LAND'],
            'NAV_GUIDED_ENABLE': cls.__constants['NAV_GUIDED_ENABLE'],
            'NAV_DELAY': cls.__constants['NAV_DELAY'],
            'NAV_PAYLOAD_PLACE': cls.__constants['NAV_PAYLOAD_PLACE'],
            'NAV_LAST': cls.__constants['NAV_LAST'],
            'NAV_SET_YAW_SPEED': cls.__constants['NAV_SET_YAW_SPEED'],
            'NAV_FENCE_RETURN_POINT': cls.__constants['NAV_FENCE_RETURN_POINT'],
            'NAV_FENCE_POLYGON_VERTEX_INCLUSION': cls.__constants['NAV_FENCE_POLYGON_VERTEX_INCLUSION'],
            'NAV_FENCE_POLYGON_VERTEX_EXCLUSION': cls.__constants['NAV_FENCE_POLYGON_VERTEX_EXCLUSION'],
            'NAV_FENCE_CIRCLE_INCLUSION': cls.__constants['NAV_FENCE_CIRCLE_INCLUSION'],
            'NAV_FENCE_CIRCLE_EXCLUSION': cls.__constants['NAV_FENCE_CIRCLE_EXCLUSION'],
            'NAV_RALLY_POINT': cls.__constants['NAV_RALLY_POINT'],
            'OBLIQUE_SURVEY': cls.__constants['OBLIQUE_SURVEY'],
            'OVERRIDE_GOTO': cls.__constants['OVERRIDE_GOTO'],
            'PANORAMA_CREATE': cls.__constants['PANORAMA_CREATE'],
            'PAYLOAD_PREPARE_DEPLOY': cls.__constants['PAYLOAD_PREPARE_DEPLOY'],
            'PAYLOAD_CONTROL_DEPLOY': cls.__constants['PAYLOAD_CONTROL_DEPLOY'],
            'PREFLIGHT_CALIBRATION': cls.__constants['PREFLIGHT_CALIBRATION'],
            'PREFLIGHT_SET_SENSOR_OFFSETS': cls.__constants['PREFLIGHT_SET_SENSOR_OFFSETS'],
            'PREFLIGHT_UAVCAN': cls.__constants['PREFLIGHT_UAVCAN'],
            'PREFLIGHT_STORAGE': cls.__constants['PREFLIGHT_STORAGE'],
            'PREFLIGHT_REBOOT_SHUTDOWN': cls.__constants['PREFLIGHT_REBOOT_SHUTDOWN'],
            'REQUEST_MESSAGE': cls.__constants['REQUEST_MESSAGE'],
            'REQUEST_PROTOCOL_VERSION': cls.__constants['REQUEST_PROTOCOL_VERSION'],
            'REQUEST_AUTOPILOT_CAPABILITIES': cls.__constants['REQUEST_AUTOPILOT_CAPABILITIES'],
            'REQUEST_CAMERA_INFORMATION': cls.__constants['REQUEST_CAMERA_INFORMATION'],
            'REQUEST_CAMERA_SETTINGS': cls.__constants['REQUEST_CAMERA_SETTINGS'],
            'REQUEST_STORAGE_INFORMATION': cls.__constants['REQUEST_STORAGE_INFORMATION'],
            'REQUEST_CAMERA_CAPTURE_STATUS': cls.__constants['REQUEST_CAMERA_CAPTURE_STATUS'],
            'REQUEST_FLIGHT_INFORMATION': cls.__constants['REQUEST_FLIGHT_INFORMATION'],
            'REQUEST_VIDEO_STREAM_INFORMATION': cls.__constants['REQUEST_VIDEO_STREAM_INFORMATION'],
            'REQUEST_VIDEO_STREAM_STATUS': cls.__constants['REQUEST_VIDEO_STREAM_STATUS'],
            'RESET_CAMERA_SETTINGS': cls.__constants['RESET_CAMERA_SETTINGS'],
            'RUN_PREARM_CHECKS': cls.__constants['RUN_PREARM_CHECKS'],
            'SET_MESSAGE_INTERVAL': cls.__constants['SET_MESSAGE_INTERVAL'],
            'SET_CAMERA_MODE': cls.__constants['SET_CAMERA_MODE'],
            'SET_CAMERA_ZOOM': cls.__constants['SET_CAMERA_ZOOM'],
            'SET_CAMERA_FOCUS': cls.__constants['SET_CAMERA_FOCUS'],
            'SET_GUIDED_SUBMODE_STANDARD': cls.__constants['SET_GUIDED_SUBMODE_STANDARD'],
            'SET_GUIDED_SUBMODE_CIRCLE': cls.__constants['SET_GUIDED_SUBMODE_CIRCLE'],
            'START_RX_PAIR': cls.__constants['START_RX_PAIR'],
            'STORAGE_FORMAT': cls.__constants['STORAGE_FORMAT'],
            'UAVCAN_GET_NODE_INFO': cls.__constants['UAVCAN_GET_NODE_INFO'],
            'VIDEO_START_CAPTURE': cls.__constants['VIDEO_START_CAPTURE'],
            'VIDEO_STOP_CAPTURE': cls.__constants['VIDEO_STOP_CAPTURE'],
            'VIDEO_START_STREAMING': cls.__constants['VIDEO_START_STREAMING'],
            'VIDEO_STOP_STREAMING': cls.__constants['VIDEO_STOP_STREAMING'],
        }

    @property
    def AIRFRAME_CONFIGURATION(self):
        """Message constant 'AIRFRAME_CONFIGURATION'."""
        return Metaclass_CommandCode.__constants['AIRFRAME_CONFIGURATION']

    @property
    def ARM_AUTHORIZATION_REQUEST(self):
        """Message constant 'ARM_AUTHORIZATION_REQUEST'."""
        return Metaclass_CommandCode.__constants['ARM_AUTHORIZATION_REQUEST']

    @property
    def CAMERA_TRACK_POINT(self):
        """Message constant 'CAMERA_TRACK_POINT'."""
        return Metaclass_CommandCode.__constants['CAMERA_TRACK_POINT']

    @property
    def CAMERA_TRACK_RECTANGLE(self):
        """Message constant 'CAMERA_TRACK_RECTANGLE'."""
        return Metaclass_CommandCode.__constants['CAMERA_TRACK_RECTANGLE']

    @property
    def CAMERA_STOP_TRACKING(self):
        """Message constant 'CAMERA_STOP_TRACKING'."""
        return Metaclass_CommandCode.__constants['CAMERA_STOP_TRACKING']

    @property
    def CAN_FORWARD(self):
        """Message constant 'CAN_FORWARD'."""
        return Metaclass_CommandCode.__constants['CAN_FORWARD']

    @property
    def COMPONENT_ARM_DISARM(self):
        """Message constant 'COMPONENT_ARM_DISARM'."""
        return Metaclass_CommandCode.__constants['COMPONENT_ARM_DISARM']

    @property
    def CONDITION_DELAY(self):
        """Message constant 'CONDITION_DELAY'."""
        return Metaclass_CommandCode.__constants['CONDITION_DELAY']

    @property
    def CONDITION_CHANGE_ALT(self):
        """Message constant 'CONDITION_CHANGE_ALT'."""
        return Metaclass_CommandCode.__constants['CONDITION_CHANGE_ALT']

    @property
    def CONDITION_DISTANCE(self):
        """Message constant 'CONDITION_DISTANCE'."""
        return Metaclass_CommandCode.__constants['CONDITION_DISTANCE']

    @property
    def CONDITION_YAW(self):
        """Message constant 'CONDITION_YAW'."""
        return Metaclass_CommandCode.__constants['CONDITION_YAW']

    @property
    def CONDITION_LAST(self):
        """Message constant 'CONDITION_LAST'."""
        return Metaclass_CommandCode.__constants['CONDITION_LAST']

    @property
    def CONTROL_HIGH_LATENCY(self):
        """Message constant 'CONTROL_HIGH_LATENCY'."""
        return Metaclass_CommandCode.__constants['CONTROL_HIGH_LATENCY']

    @property
    def DO_FOLLOW(self):
        """Message constant 'DO_FOLLOW'."""
        return Metaclass_CommandCode.__constants['DO_FOLLOW']

    @property
    def DO_FOLLOW_REPOSITION(self):
        """Message constant 'DO_FOLLOW_REPOSITION'."""
        return Metaclass_CommandCode.__constants['DO_FOLLOW_REPOSITION']

    @property
    def DO_SET_MODE(self):
        """Message constant 'DO_SET_MODE'."""
        return Metaclass_CommandCode.__constants['DO_SET_MODE']

    @property
    def DO_JUMP(self):
        """Message constant 'DO_JUMP'."""
        return Metaclass_CommandCode.__constants['DO_JUMP']

    @property
    def DO_CHANGE_SPEED(self):
        """Message constant 'DO_CHANGE_SPEED'."""
        return Metaclass_CommandCode.__constants['DO_CHANGE_SPEED']

    @property
    def DO_SET_HOME(self):
        """Message constant 'DO_SET_HOME'."""
        return Metaclass_CommandCode.__constants['DO_SET_HOME']

    @property
    def DO_SET_PARAMETER(self):
        """Message constant 'DO_SET_PARAMETER'."""
        return Metaclass_CommandCode.__constants['DO_SET_PARAMETER']

    @property
    def DO_SET_RELAY(self):
        """Message constant 'DO_SET_RELAY'."""
        return Metaclass_CommandCode.__constants['DO_SET_RELAY']

    @property
    def DO_REPEAT_RELAY(self):
        """Message constant 'DO_REPEAT_RELAY'."""
        return Metaclass_CommandCode.__constants['DO_REPEAT_RELAY']

    @property
    def DO_SET_SERVO(self):
        """Message constant 'DO_SET_SERVO'."""
        return Metaclass_CommandCode.__constants['DO_SET_SERVO']

    @property
    def DO_REPEAT_SERVO(self):
        """Message constant 'DO_REPEAT_SERVO'."""
        return Metaclass_CommandCode.__constants['DO_REPEAT_SERVO']

    @property
    def DO_FLIGHTTERMINATION(self):
        """Message constant 'DO_FLIGHTTERMINATION'."""
        return Metaclass_CommandCode.__constants['DO_FLIGHTTERMINATION']

    @property
    def DO_CHANGE_ALTITUDE(self):
        """Message constant 'DO_CHANGE_ALTITUDE'."""
        return Metaclass_CommandCode.__constants['DO_CHANGE_ALTITUDE']

    @property
    def DO_LAND_START(self):
        """Message constant 'DO_LAND_START'."""
        return Metaclass_CommandCode.__constants['DO_LAND_START']

    @property
    def DO_RALLY_LAND(self):
        """Message constant 'DO_RALLY_LAND'."""
        return Metaclass_CommandCode.__constants['DO_RALLY_LAND']

    @property
    def DO_GO_AROUND(self):
        """Message constant 'DO_GO_AROUND'."""
        return Metaclass_CommandCode.__constants['DO_GO_AROUND']

    @property
    def DO_REPOSITION(self):
        """Message constant 'DO_REPOSITION'."""
        return Metaclass_CommandCode.__constants['DO_REPOSITION']

    @property
    def DO_PAUSE_CONTINUE(self):
        """Message constant 'DO_PAUSE_CONTINUE'."""
        return Metaclass_CommandCode.__constants['DO_PAUSE_CONTINUE']

    @property
    def DO_SET_REVERSE(self):
        """Message constant 'DO_SET_REVERSE'."""
        return Metaclass_CommandCode.__constants['DO_SET_REVERSE']

    @property
    def DO_SET_ROI_LOCATION(self):
        """Message constant 'DO_SET_ROI_LOCATION'."""
        return Metaclass_CommandCode.__constants['DO_SET_ROI_LOCATION']

    @property
    def DO_SET_ROI_WPNEXT_OFFSET(self):
        """Message constant 'DO_SET_ROI_WPNEXT_OFFSET'."""
        return Metaclass_CommandCode.__constants['DO_SET_ROI_WPNEXT_OFFSET']

    @property
    def DO_SET_ROI_NONE(self):
        """Message constant 'DO_SET_ROI_NONE'."""
        return Metaclass_CommandCode.__constants['DO_SET_ROI_NONE']

    @property
    def DO_SET_ROI_SYSID(self):
        """Message constant 'DO_SET_ROI_SYSID'."""
        return Metaclass_CommandCode.__constants['DO_SET_ROI_SYSID']

    @property
    def DO_CONTROL_VIDEO(self):
        """Message constant 'DO_CONTROL_VIDEO'."""
        return Metaclass_CommandCode.__constants['DO_CONTROL_VIDEO']

    @property
    def DO_SET_ROI(self):
        """Message constant 'DO_SET_ROI'."""
        return Metaclass_CommandCode.__constants['DO_SET_ROI']

    @property
    def DO_DIGICAM_CONFIGURE(self):
        """Message constant 'DO_DIGICAM_CONFIGURE'."""
        return Metaclass_CommandCode.__constants['DO_DIGICAM_CONFIGURE']

    @property
    def DO_DIGICAM_CONTROL(self):
        """Message constant 'DO_DIGICAM_CONTROL'."""
        return Metaclass_CommandCode.__constants['DO_DIGICAM_CONTROL']

    @property
    def DO_MOUNT_CONFIGURE(self):
        """Message constant 'DO_MOUNT_CONFIGURE'."""
        return Metaclass_CommandCode.__constants['DO_MOUNT_CONFIGURE']

    @property
    def DO_MOUNT_CONTROL(self):
        """Message constant 'DO_MOUNT_CONTROL'."""
        return Metaclass_CommandCode.__constants['DO_MOUNT_CONTROL']

    @property
    def DO_SET_CAM_TRIGG_DIST(self):
        """Message constant 'DO_SET_CAM_TRIGG_DIST'."""
        return Metaclass_CommandCode.__constants['DO_SET_CAM_TRIGG_DIST']

    @property
    def DO_FENCE_ENABLE(self):
        """Message constant 'DO_FENCE_ENABLE'."""
        return Metaclass_CommandCode.__constants['DO_FENCE_ENABLE']

    @property
    def DO_PARACHUTE(self):
        """Message constant 'DO_PARACHUTE'."""
        return Metaclass_CommandCode.__constants['DO_PARACHUTE']

    @property
    def DO_MOTOR_TEST(self):
        """Message constant 'DO_MOTOR_TEST'."""
        return Metaclass_CommandCode.__constants['DO_MOTOR_TEST']

    @property
    def DO_INVERTED_FLIGHT(self):
        """Message constant 'DO_INVERTED_FLIGHT'."""
        return Metaclass_CommandCode.__constants['DO_INVERTED_FLIGHT']

    @property
    def DO_GRIPPER(self):
        """Message constant 'DO_GRIPPER'."""
        return Metaclass_CommandCode.__constants['DO_GRIPPER']

    @property
    def DO_AUTOTUNE_ENABLE(self):
        """Message constant 'DO_AUTOTUNE_ENABLE'."""
        return Metaclass_CommandCode.__constants['DO_AUTOTUNE_ENABLE']

    @property
    def DO_SET_CAM_TRIGG_INTERVAL(self):
        """Message constant 'DO_SET_CAM_TRIGG_INTERVAL'."""
        return Metaclass_CommandCode.__constants['DO_SET_CAM_TRIGG_INTERVAL']

    @property
    def DO_MOUNT_CONTROL_QUAT(self):
        """Message constant 'DO_MOUNT_CONTROL_QUAT'."""
        return Metaclass_CommandCode.__constants['DO_MOUNT_CONTROL_QUAT']

    @property
    def DO_GUIDED_MASTER(self):
        """Message constant 'DO_GUIDED_MASTER'."""
        return Metaclass_CommandCode.__constants['DO_GUIDED_MASTER']

    @property
    def DO_GUIDED_LIMITS(self):
        """Message constant 'DO_GUIDED_LIMITS'."""
        return Metaclass_CommandCode.__constants['DO_GUIDED_LIMITS']

    @property
    def DO_ENGINE_CONTROL(self):
        """Message constant 'DO_ENGINE_CONTROL'."""
        return Metaclass_CommandCode.__constants['DO_ENGINE_CONTROL']

    @property
    def DO_SET_MISSION_CURRENT(self):
        """Message constant 'DO_SET_MISSION_CURRENT'."""
        return Metaclass_CommandCode.__constants['DO_SET_MISSION_CURRENT']

    @property
    def DO_LAST(self):
        """Message constant 'DO_LAST'."""
        return Metaclass_CommandCode.__constants['DO_LAST']

    @property
    def DO_JUMP_TAG(self):
        """Message constant 'DO_JUMP_TAG'."""
        return Metaclass_CommandCode.__constants['DO_JUMP_TAG']

    @property
    def DO_GIMBAL_MANAGER_PITCHYAW(self):
        """Message constant 'DO_GIMBAL_MANAGER_PITCHYAW'."""
        return Metaclass_CommandCode.__constants['DO_GIMBAL_MANAGER_PITCHYAW']

    @property
    def DO_GIMBAL_MANAGER_CONFIGURE(self):
        """Message constant 'DO_GIMBAL_MANAGER_CONFIGURE'."""
        return Metaclass_CommandCode.__constants['DO_GIMBAL_MANAGER_CONFIGURE']

    @property
    def DO_TRIGGER_CONTROL(self):
        """Message constant 'DO_TRIGGER_CONTROL'."""
        return Metaclass_CommandCode.__constants['DO_TRIGGER_CONTROL']

    @property
    def DO_VTOL_TRANSITION(self):
        """Message constant 'DO_VTOL_TRANSITION'."""
        return Metaclass_CommandCode.__constants['DO_VTOL_TRANSITION']

    @property
    def DO_ADSB_OUT_IDENT(self):
        """Message constant 'DO_ADSB_OUT_IDENT'."""
        return Metaclass_CommandCode.__constants['DO_ADSB_OUT_IDENT']

    @property
    def DO_WINCH(self):
        """Message constant 'DO_WINCH'."""
        return Metaclass_CommandCode.__constants['DO_WINCH']

    @property
    def FIXED_MAG_CAL_YAW(self):
        """Message constant 'FIXED_MAG_CAL_YAW'."""
        return Metaclass_CommandCode.__constants['FIXED_MAG_CAL_YAW']

    @property
    def GET_HOME_POSITION(self):
        """Message constant 'GET_HOME_POSITION'."""
        return Metaclass_CommandCode.__constants['GET_HOME_POSITION']

    @property
    def GET_MESSAGE_INTERVAL(self):
        """Message constant 'GET_MESSAGE_INTERVAL'."""
        return Metaclass_CommandCode.__constants['GET_MESSAGE_INTERVAL']

    @property
    def IMAGE_START_CAPTURE(self):
        """Message constant 'IMAGE_START_CAPTURE'."""
        return Metaclass_CommandCode.__constants['IMAGE_START_CAPTURE']

    @property
    def IMAGE_STOP_CAPTURE(self):
        """Message constant 'IMAGE_STOP_CAPTURE'."""
        return Metaclass_CommandCode.__constants['IMAGE_STOP_CAPTURE']

    @property
    def JUMP_TAG(self):
        """Message constant 'JUMP_TAG'."""
        return Metaclass_CommandCode.__constants['JUMP_TAG']

    @property
    def LOGGING_START(self):
        """Message constant 'LOGGING_START'."""
        return Metaclass_CommandCode.__constants['LOGGING_START']

    @property
    def LOGGING_STOP(self):
        """Message constant 'LOGGING_STOP'."""
        return Metaclass_CommandCode.__constants['LOGGING_STOP']

    @property
    def MISSION_START(self):
        """Message constant 'MISSION_START'."""
        return Metaclass_CommandCode.__constants['MISSION_START']

    @property
    def NAV_WAYPOINT(self):
        """Message constant 'NAV_WAYPOINT'."""
        return Metaclass_CommandCode.__constants['NAV_WAYPOINT']

    @property
    def NAV_LOITER_UNLIM(self):
        """Message constant 'NAV_LOITER_UNLIM'."""
        return Metaclass_CommandCode.__constants['NAV_LOITER_UNLIM']

    @property
    def NAV_LOITER_TURNS(self):
        """Message constant 'NAV_LOITER_TURNS'."""
        return Metaclass_CommandCode.__constants['NAV_LOITER_TURNS']

    @property
    def NAV_LOITER_TIME(self):
        """Message constant 'NAV_LOITER_TIME'."""
        return Metaclass_CommandCode.__constants['NAV_LOITER_TIME']

    @property
    def NAV_RETURN_TO_LAUNCH(self):
        """Message constant 'NAV_RETURN_TO_LAUNCH'."""
        return Metaclass_CommandCode.__constants['NAV_RETURN_TO_LAUNCH']

    @property
    def NAV_LAND(self):
        """Message constant 'NAV_LAND'."""
        return Metaclass_CommandCode.__constants['NAV_LAND']

    @property
    def NAV_TAKEOFF(self):
        """Message constant 'NAV_TAKEOFF'."""
        return Metaclass_CommandCode.__constants['NAV_TAKEOFF']

    @property
    def NAV_LAND_LOCAL(self):
        """Message constant 'NAV_LAND_LOCAL'."""
        return Metaclass_CommandCode.__constants['NAV_LAND_LOCAL']

    @property
    def NAV_TAKEOFF_LOCAL(self):
        """Message constant 'NAV_TAKEOFF_LOCAL'."""
        return Metaclass_CommandCode.__constants['NAV_TAKEOFF_LOCAL']

    @property
    def NAV_FOLLOW(self):
        """Message constant 'NAV_FOLLOW'."""
        return Metaclass_CommandCode.__constants['NAV_FOLLOW']

    @property
    def NAV_CONTINUE_AND_CHANGE_ALT(self):
        """Message constant 'NAV_CONTINUE_AND_CHANGE_ALT'."""
        return Metaclass_CommandCode.__constants['NAV_CONTINUE_AND_CHANGE_ALT']

    @property
    def NAV_LOITER_TO_ALT(self):
        """Message constant 'NAV_LOITER_TO_ALT'."""
        return Metaclass_CommandCode.__constants['NAV_LOITER_TO_ALT']

    @property
    def NAV_ROI(self):
        """Message constant 'NAV_ROI'."""
        return Metaclass_CommandCode.__constants['NAV_ROI']

    @property
    def NAV_PATHPLANNING(self):
        """Message constant 'NAV_PATHPLANNING'."""
        return Metaclass_CommandCode.__constants['NAV_PATHPLANNING']

    @property
    def NAV_SPLINE_WAYPOINT(self):
        """Message constant 'NAV_SPLINE_WAYPOINT'."""
        return Metaclass_CommandCode.__constants['NAV_SPLINE_WAYPOINT']

    @property
    def NAV_VTOL_TAKEOFF(self):
        """Message constant 'NAV_VTOL_TAKEOFF'."""
        return Metaclass_CommandCode.__constants['NAV_VTOL_TAKEOFF']

    @property
    def NAV_VTOL_LAND(self):
        """Message constant 'NAV_VTOL_LAND'."""
        return Metaclass_CommandCode.__constants['NAV_VTOL_LAND']

    @property
    def NAV_GUIDED_ENABLE(self):
        """Message constant 'NAV_GUIDED_ENABLE'."""
        return Metaclass_CommandCode.__constants['NAV_GUIDED_ENABLE']

    @property
    def NAV_DELAY(self):
        """Message constant 'NAV_DELAY'."""
        return Metaclass_CommandCode.__constants['NAV_DELAY']

    @property
    def NAV_PAYLOAD_PLACE(self):
        """Message constant 'NAV_PAYLOAD_PLACE'."""
        return Metaclass_CommandCode.__constants['NAV_PAYLOAD_PLACE']

    @property
    def NAV_LAST(self):
        """Message constant 'NAV_LAST'."""
        return Metaclass_CommandCode.__constants['NAV_LAST']

    @property
    def NAV_SET_YAW_SPEED(self):
        """Message constant 'NAV_SET_YAW_SPEED'."""
        return Metaclass_CommandCode.__constants['NAV_SET_YAW_SPEED']

    @property
    def NAV_FENCE_RETURN_POINT(self):
        """Message constant 'NAV_FENCE_RETURN_POINT'."""
        return Metaclass_CommandCode.__constants['NAV_FENCE_RETURN_POINT']

    @property
    def NAV_FENCE_POLYGON_VERTEX_INCLUSION(self):
        """Message constant 'NAV_FENCE_POLYGON_VERTEX_INCLUSION'."""
        return Metaclass_CommandCode.__constants['NAV_FENCE_POLYGON_VERTEX_INCLUSION']

    @property
    def NAV_FENCE_POLYGON_VERTEX_EXCLUSION(self):
        """Message constant 'NAV_FENCE_POLYGON_VERTEX_EXCLUSION'."""
        return Metaclass_CommandCode.__constants['NAV_FENCE_POLYGON_VERTEX_EXCLUSION']

    @property
    def NAV_FENCE_CIRCLE_INCLUSION(self):
        """Message constant 'NAV_FENCE_CIRCLE_INCLUSION'."""
        return Metaclass_CommandCode.__constants['NAV_FENCE_CIRCLE_INCLUSION']

    @property
    def NAV_FENCE_CIRCLE_EXCLUSION(self):
        """Message constant 'NAV_FENCE_CIRCLE_EXCLUSION'."""
        return Metaclass_CommandCode.__constants['NAV_FENCE_CIRCLE_EXCLUSION']

    @property
    def NAV_RALLY_POINT(self):
        """Message constant 'NAV_RALLY_POINT'."""
        return Metaclass_CommandCode.__constants['NAV_RALLY_POINT']

    @property
    def OBLIQUE_SURVEY(self):
        """Message constant 'OBLIQUE_SURVEY'."""
        return Metaclass_CommandCode.__constants['OBLIQUE_SURVEY']

    @property
    def OVERRIDE_GOTO(self):
        """Message constant 'OVERRIDE_GOTO'."""
        return Metaclass_CommandCode.__constants['OVERRIDE_GOTO']

    @property
    def PANORAMA_CREATE(self):
        """Message constant 'PANORAMA_CREATE'."""
        return Metaclass_CommandCode.__constants['PANORAMA_CREATE']

    @property
    def PAYLOAD_PREPARE_DEPLOY(self):
        """Message constant 'PAYLOAD_PREPARE_DEPLOY'."""
        return Metaclass_CommandCode.__constants['PAYLOAD_PREPARE_DEPLOY']

    @property
    def PAYLOAD_CONTROL_DEPLOY(self):
        """Message constant 'PAYLOAD_CONTROL_DEPLOY'."""
        return Metaclass_CommandCode.__constants['PAYLOAD_CONTROL_DEPLOY']

    @property
    def PREFLIGHT_CALIBRATION(self):
        """Message constant 'PREFLIGHT_CALIBRATION'."""
        return Metaclass_CommandCode.__constants['PREFLIGHT_CALIBRATION']

    @property
    def PREFLIGHT_SET_SENSOR_OFFSETS(self):
        """Message constant 'PREFLIGHT_SET_SENSOR_OFFSETS'."""
        return Metaclass_CommandCode.__constants['PREFLIGHT_SET_SENSOR_OFFSETS']

    @property
    def PREFLIGHT_UAVCAN(self):
        """Message constant 'PREFLIGHT_UAVCAN'."""
        return Metaclass_CommandCode.__constants['PREFLIGHT_UAVCAN']

    @property
    def PREFLIGHT_STORAGE(self):
        """Message constant 'PREFLIGHT_STORAGE'."""
        return Metaclass_CommandCode.__constants['PREFLIGHT_STORAGE']

    @property
    def PREFLIGHT_REBOOT_SHUTDOWN(self):
        """Message constant 'PREFLIGHT_REBOOT_SHUTDOWN'."""
        return Metaclass_CommandCode.__constants['PREFLIGHT_REBOOT_SHUTDOWN']

    @property
    def REQUEST_MESSAGE(self):
        """Message constant 'REQUEST_MESSAGE'."""
        return Metaclass_CommandCode.__constants['REQUEST_MESSAGE']

    @property
    def REQUEST_PROTOCOL_VERSION(self):
        """Message constant 'REQUEST_PROTOCOL_VERSION'."""
        return Metaclass_CommandCode.__constants['REQUEST_PROTOCOL_VERSION']

    @property
    def REQUEST_AUTOPILOT_CAPABILITIES(self):
        """Message constant 'REQUEST_AUTOPILOT_CAPABILITIES'."""
        return Metaclass_CommandCode.__constants['REQUEST_AUTOPILOT_CAPABILITIES']

    @property
    def REQUEST_CAMERA_INFORMATION(self):
        """Message constant 'REQUEST_CAMERA_INFORMATION'."""
        return Metaclass_CommandCode.__constants['REQUEST_CAMERA_INFORMATION']

    @property
    def REQUEST_CAMERA_SETTINGS(self):
        """Message constant 'REQUEST_CAMERA_SETTINGS'."""
        return Metaclass_CommandCode.__constants['REQUEST_CAMERA_SETTINGS']

    @property
    def REQUEST_STORAGE_INFORMATION(self):
        """Message constant 'REQUEST_STORAGE_INFORMATION'."""
        return Metaclass_CommandCode.__constants['REQUEST_STORAGE_INFORMATION']

    @property
    def REQUEST_CAMERA_CAPTURE_STATUS(self):
        """Message constant 'REQUEST_CAMERA_CAPTURE_STATUS'."""
        return Metaclass_CommandCode.__constants['REQUEST_CAMERA_CAPTURE_STATUS']

    @property
    def REQUEST_FLIGHT_INFORMATION(self):
        """Message constant 'REQUEST_FLIGHT_INFORMATION'."""
        return Metaclass_CommandCode.__constants['REQUEST_FLIGHT_INFORMATION']

    @property
    def REQUEST_VIDEO_STREAM_INFORMATION(self):
        """Message constant 'REQUEST_VIDEO_STREAM_INFORMATION'."""
        return Metaclass_CommandCode.__constants['REQUEST_VIDEO_STREAM_INFORMATION']

    @property
    def REQUEST_VIDEO_STREAM_STATUS(self):
        """Message constant 'REQUEST_VIDEO_STREAM_STATUS'."""
        return Metaclass_CommandCode.__constants['REQUEST_VIDEO_STREAM_STATUS']

    @property
    def RESET_CAMERA_SETTINGS(self):
        """Message constant 'RESET_CAMERA_SETTINGS'."""
        return Metaclass_CommandCode.__constants['RESET_CAMERA_SETTINGS']

    @property
    def RUN_PREARM_CHECKS(self):
        """Message constant 'RUN_PREARM_CHECKS'."""
        return Metaclass_CommandCode.__constants['RUN_PREARM_CHECKS']

    @property
    def SET_MESSAGE_INTERVAL(self):
        """Message constant 'SET_MESSAGE_INTERVAL'."""
        return Metaclass_CommandCode.__constants['SET_MESSAGE_INTERVAL']

    @property
    def SET_CAMERA_MODE(self):
        """Message constant 'SET_CAMERA_MODE'."""
        return Metaclass_CommandCode.__constants['SET_CAMERA_MODE']

    @property
    def SET_CAMERA_ZOOM(self):
        """Message constant 'SET_CAMERA_ZOOM'."""
        return Metaclass_CommandCode.__constants['SET_CAMERA_ZOOM']

    @property
    def SET_CAMERA_FOCUS(self):
        """Message constant 'SET_CAMERA_FOCUS'."""
        return Metaclass_CommandCode.__constants['SET_CAMERA_FOCUS']

    @property
    def SET_GUIDED_SUBMODE_STANDARD(self):
        """Message constant 'SET_GUIDED_SUBMODE_STANDARD'."""
        return Metaclass_CommandCode.__constants['SET_GUIDED_SUBMODE_STANDARD']

    @property
    def SET_GUIDED_SUBMODE_CIRCLE(self):
        """Message constant 'SET_GUIDED_SUBMODE_CIRCLE'."""
        return Metaclass_CommandCode.__constants['SET_GUIDED_SUBMODE_CIRCLE']

    @property
    def START_RX_PAIR(self):
        """Message constant 'START_RX_PAIR'."""
        return Metaclass_CommandCode.__constants['START_RX_PAIR']

    @property
    def STORAGE_FORMAT(self):
        """Message constant 'STORAGE_FORMAT'."""
        return Metaclass_CommandCode.__constants['STORAGE_FORMAT']

    @property
    def UAVCAN_GET_NODE_INFO(self):
        """Message constant 'UAVCAN_GET_NODE_INFO'."""
        return Metaclass_CommandCode.__constants['UAVCAN_GET_NODE_INFO']

    @property
    def VIDEO_START_CAPTURE(self):
        """Message constant 'VIDEO_START_CAPTURE'."""
        return Metaclass_CommandCode.__constants['VIDEO_START_CAPTURE']

    @property
    def VIDEO_STOP_CAPTURE(self):
        """Message constant 'VIDEO_STOP_CAPTURE'."""
        return Metaclass_CommandCode.__constants['VIDEO_STOP_CAPTURE']

    @property
    def VIDEO_START_STREAMING(self):
        """Message constant 'VIDEO_START_STREAMING'."""
        return Metaclass_CommandCode.__constants['VIDEO_START_STREAMING']

    @property
    def VIDEO_STOP_STREAMING(self):
        """Message constant 'VIDEO_STOP_STREAMING'."""
        return Metaclass_CommandCode.__constants['VIDEO_STOP_STREAMING']


class CommandCode(metaclass=Metaclass_CommandCode):
    """
    Message class 'CommandCode'.

    Constants:
      AIRFRAME_CONFIGURATION
      ARM_AUTHORIZATION_REQUEST
      CAMERA_TRACK_POINT
      CAMERA_TRACK_RECTANGLE
      CAMERA_STOP_TRACKING
      CAN_FORWARD
      COMPONENT_ARM_DISARM
      CONDITION_DELAY
      CONDITION_CHANGE_ALT
      CONDITION_DISTANCE
      CONDITION_YAW
      CONDITION_LAST
      CONTROL_HIGH_LATENCY
      DO_FOLLOW
      DO_FOLLOW_REPOSITION
      DO_SET_MODE
      DO_JUMP
      DO_CHANGE_SPEED
      DO_SET_HOME
      DO_SET_PARAMETER
      DO_SET_RELAY
      DO_REPEAT_RELAY
      DO_SET_SERVO
      DO_REPEAT_SERVO
      DO_FLIGHTTERMINATION
      DO_CHANGE_ALTITUDE
      DO_LAND_START
      DO_RALLY_LAND
      DO_GO_AROUND
      DO_REPOSITION
      DO_PAUSE_CONTINUE
      DO_SET_REVERSE
      DO_SET_ROI_LOCATION
      DO_SET_ROI_WPNEXT_OFFSET
      DO_SET_ROI_NONE
      DO_SET_ROI_SYSID
      DO_CONTROL_VIDEO
      DO_SET_ROI
      DO_DIGICAM_CONFIGURE
      DO_DIGICAM_CONTROL
      DO_MOUNT_CONFIGURE
      DO_MOUNT_CONTROL
      DO_SET_CAM_TRIGG_DIST
      DO_FENCE_ENABLE
      DO_PARACHUTE
      DO_MOTOR_TEST
      DO_INVERTED_FLIGHT
      DO_GRIPPER
      DO_AUTOTUNE_ENABLE
      DO_SET_CAM_TRIGG_INTERVAL
      DO_MOUNT_CONTROL_QUAT
      DO_GUIDED_MASTER
      DO_GUIDED_LIMITS
      DO_ENGINE_CONTROL
      DO_SET_MISSION_CURRENT
      DO_LAST
      DO_JUMP_TAG
      DO_GIMBAL_MANAGER_PITCHYAW
      DO_GIMBAL_MANAGER_CONFIGURE
      DO_TRIGGER_CONTROL
      DO_VTOL_TRANSITION
      DO_ADSB_OUT_IDENT
      DO_WINCH
      FIXED_MAG_CAL_YAW
      GET_HOME_POSITION
      GET_MESSAGE_INTERVAL
      IMAGE_START_CAPTURE
      IMAGE_STOP_CAPTURE
      JUMP_TAG
      LOGGING_START
      LOGGING_STOP
      MISSION_START
      NAV_WAYPOINT
      NAV_LOITER_UNLIM
      NAV_LOITER_TURNS
      NAV_LOITER_TIME
      NAV_RETURN_TO_LAUNCH
      NAV_LAND
      NAV_TAKEOFF
      NAV_LAND_LOCAL
      NAV_TAKEOFF_LOCAL
      NAV_FOLLOW
      NAV_CONTINUE_AND_CHANGE_ALT
      NAV_LOITER_TO_ALT
      NAV_ROI
      NAV_PATHPLANNING
      NAV_SPLINE_WAYPOINT
      NAV_VTOL_TAKEOFF
      NAV_VTOL_LAND
      NAV_GUIDED_ENABLE
      NAV_DELAY
      NAV_PAYLOAD_PLACE
      NAV_LAST
      NAV_SET_YAW_SPEED
      NAV_FENCE_RETURN_POINT
      NAV_FENCE_POLYGON_VERTEX_INCLUSION
      NAV_FENCE_POLYGON_VERTEX_EXCLUSION
      NAV_FENCE_CIRCLE_INCLUSION
      NAV_FENCE_CIRCLE_EXCLUSION
      NAV_RALLY_POINT
      OBLIQUE_SURVEY
      OVERRIDE_GOTO
      PANORAMA_CREATE
      PAYLOAD_PREPARE_DEPLOY
      PAYLOAD_CONTROL_DEPLOY
      PREFLIGHT_CALIBRATION
      PREFLIGHT_SET_SENSOR_OFFSETS
      PREFLIGHT_UAVCAN
      PREFLIGHT_STORAGE
      PREFLIGHT_REBOOT_SHUTDOWN
      REQUEST_MESSAGE
      REQUEST_PROTOCOL_VERSION
      REQUEST_AUTOPILOT_CAPABILITIES
      REQUEST_CAMERA_INFORMATION
      REQUEST_CAMERA_SETTINGS
      REQUEST_STORAGE_INFORMATION
      REQUEST_CAMERA_CAPTURE_STATUS
      REQUEST_FLIGHT_INFORMATION
      REQUEST_VIDEO_STREAM_INFORMATION
      REQUEST_VIDEO_STREAM_STATUS
      RESET_CAMERA_SETTINGS
      RUN_PREARM_CHECKS
      SET_MESSAGE_INTERVAL
      SET_CAMERA_MODE
      SET_CAMERA_ZOOM
      SET_CAMERA_FOCUS
      SET_GUIDED_SUBMODE_STANDARD
      SET_GUIDED_SUBMODE_CIRCLE
      START_RX_PAIR
      STORAGE_FORMAT
      UAVCAN_GET_NODE_INFO
      VIDEO_START_CAPTURE
      VIDEO_STOP_CAPTURE
      VIDEO_START_STREAMING
      VIDEO_STOP_STREAMING
    """

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)
