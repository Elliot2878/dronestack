# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/GimbalDeviceAttitudeStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalDeviceAttitudeStatus(type):
    """Metaclass of message 'GimbalDeviceAttitudeStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FLAGS_RETRACT': 1,
        'FLAGS_NEUTRAL': 2,
        'FLAGS_ROLL_LOCK': 4,
        'FLAGS_PITCH_LOCK': 8,
        'FLAGS_YAW_LOCK': 16,
        'ERROR_FLAGS_AT_ROLL_LIMIT': 1,
        'ERROR_FLAGS_AT_PITCH_LIMIT': 2,
        'ERROR_FLAGS_AT_YAW_LIMIT': 4,
        'ERROR_FLAGS_ENCODER_ERROR': 8,
        'ERROR_FLAGS_POWER_ERROR': 16,
        'ERROR_FLAGS_MOTOR_ERROR': 32,
        'ERROR_FLAGS_SOFTWARE_ERROR': 64,
        'ERROR_FLAGS_COMMS_ERROR': 128,
        'ERROR_FLAGS_CALIBRATION_RUNNING': 256,
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
                'mavros_msgs.msg.GimbalDeviceAttitudeStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gimbal_device_attitude_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gimbal_device_attitude_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gimbal_device_attitude_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gimbal_device_attitude_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gimbal_device_attitude_status

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FLAGS_RETRACT': cls.__constants['FLAGS_RETRACT'],
            'FLAGS_NEUTRAL': cls.__constants['FLAGS_NEUTRAL'],
            'FLAGS_ROLL_LOCK': cls.__constants['FLAGS_ROLL_LOCK'],
            'FLAGS_PITCH_LOCK': cls.__constants['FLAGS_PITCH_LOCK'],
            'FLAGS_YAW_LOCK': cls.__constants['FLAGS_YAW_LOCK'],
            'ERROR_FLAGS_AT_ROLL_LIMIT': cls.__constants['ERROR_FLAGS_AT_ROLL_LIMIT'],
            'ERROR_FLAGS_AT_PITCH_LIMIT': cls.__constants['ERROR_FLAGS_AT_PITCH_LIMIT'],
            'ERROR_FLAGS_AT_YAW_LIMIT': cls.__constants['ERROR_FLAGS_AT_YAW_LIMIT'],
            'ERROR_FLAGS_ENCODER_ERROR': cls.__constants['ERROR_FLAGS_ENCODER_ERROR'],
            'ERROR_FLAGS_POWER_ERROR': cls.__constants['ERROR_FLAGS_POWER_ERROR'],
            'ERROR_FLAGS_MOTOR_ERROR': cls.__constants['ERROR_FLAGS_MOTOR_ERROR'],
            'ERROR_FLAGS_SOFTWARE_ERROR': cls.__constants['ERROR_FLAGS_SOFTWARE_ERROR'],
            'ERROR_FLAGS_COMMS_ERROR': cls.__constants['ERROR_FLAGS_COMMS_ERROR'],
            'ERROR_FLAGS_CALIBRATION_RUNNING': cls.__constants['ERROR_FLAGS_CALIBRATION_RUNNING'],
        }

    @property
    def FLAGS_RETRACT(self):
        """Message constant 'FLAGS_RETRACT'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['FLAGS_RETRACT']

    @property
    def FLAGS_NEUTRAL(self):
        """Message constant 'FLAGS_NEUTRAL'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['FLAGS_NEUTRAL']

    @property
    def FLAGS_ROLL_LOCK(self):
        """Message constant 'FLAGS_ROLL_LOCK'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['FLAGS_ROLL_LOCK']

    @property
    def FLAGS_PITCH_LOCK(self):
        """Message constant 'FLAGS_PITCH_LOCK'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['FLAGS_PITCH_LOCK']

    @property
    def FLAGS_YAW_LOCK(self):
        """Message constant 'FLAGS_YAW_LOCK'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['FLAGS_YAW_LOCK']

    @property
    def ERROR_FLAGS_AT_ROLL_LIMIT(self):
        """Message constant 'ERROR_FLAGS_AT_ROLL_LIMIT'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_AT_ROLL_LIMIT']

    @property
    def ERROR_FLAGS_AT_PITCH_LIMIT(self):
        """Message constant 'ERROR_FLAGS_AT_PITCH_LIMIT'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_AT_PITCH_LIMIT']

    @property
    def ERROR_FLAGS_AT_YAW_LIMIT(self):
        """Message constant 'ERROR_FLAGS_AT_YAW_LIMIT'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_AT_YAW_LIMIT']

    @property
    def ERROR_FLAGS_ENCODER_ERROR(self):
        """Message constant 'ERROR_FLAGS_ENCODER_ERROR'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_ENCODER_ERROR']

    @property
    def ERROR_FLAGS_POWER_ERROR(self):
        """Message constant 'ERROR_FLAGS_POWER_ERROR'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_POWER_ERROR']

    @property
    def ERROR_FLAGS_MOTOR_ERROR(self):
        """Message constant 'ERROR_FLAGS_MOTOR_ERROR'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_MOTOR_ERROR']

    @property
    def ERROR_FLAGS_SOFTWARE_ERROR(self):
        """Message constant 'ERROR_FLAGS_SOFTWARE_ERROR'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_SOFTWARE_ERROR']

    @property
    def ERROR_FLAGS_COMMS_ERROR(self):
        """Message constant 'ERROR_FLAGS_COMMS_ERROR'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_COMMS_ERROR']

    @property
    def ERROR_FLAGS_CALIBRATION_RUNNING(self):
        """Message constant 'ERROR_FLAGS_CALIBRATION_RUNNING'."""
        return Metaclass_GimbalDeviceAttitudeStatus.__constants['ERROR_FLAGS_CALIBRATION_RUNNING']


class GimbalDeviceAttitudeStatus(metaclass=Metaclass_GimbalDeviceAttitudeStatus):
    """
    Message class 'GimbalDeviceAttitudeStatus'.

    Constants:
      FLAGS_RETRACT
      FLAGS_NEUTRAL
      FLAGS_ROLL_LOCK
      FLAGS_PITCH_LOCK
      FLAGS_YAW_LOCK
      ERROR_FLAGS_AT_ROLL_LIMIT
      ERROR_FLAGS_AT_PITCH_LIMIT
      ERROR_FLAGS_AT_YAW_LIMIT
      ERROR_FLAGS_ENCODER_ERROR
      ERROR_FLAGS_POWER_ERROR
      ERROR_FLAGS_MOTOR_ERROR
      ERROR_FLAGS_SOFTWARE_ERROR
      ERROR_FLAGS_COMMS_ERROR
      ERROR_FLAGS_CALIBRATION_RUNNING
    """

    __slots__ = [
        '_header',
        '_target_system',
        '_target_component',
        '_flags',
        '_q',
        '_angular_velocity_x',
        '_angular_velocity_y',
        '_angular_velocity_z',
        '_failure_flags',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'target_system': 'uint8',
        'target_component': 'uint8',
        'flags': 'uint16',
        'q': 'geometry_msgs/Quaternion',
        'angular_velocity_x': 'float',
        'angular_velocity_y': 'float',
        'angular_velocity_z': 'float',
        'failure_flags': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.target_system = kwargs.get('target_system', int())
        self.target_component = kwargs.get('target_component', int())
        self.flags = kwargs.get('flags', int())
        from geometry_msgs.msg import Quaternion
        self.q = kwargs.get('q', Quaternion())
        self.angular_velocity_x = kwargs.get('angular_velocity_x', float())
        self.angular_velocity_y = kwargs.get('angular_velocity_y', float())
        self.angular_velocity_z = kwargs.get('angular_velocity_z', float())
        self.failure_flags = kwargs.get('failure_flags', int())

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
        if self.header != other.header:
            return False
        if self.target_system != other.target_system:
            return False
        if self.target_component != other.target_component:
            return False
        if self.flags != other.flags:
            return False
        if self.q != other.q:
            return False
        if self.angular_velocity_x != other.angular_velocity_x:
            return False
        if self.angular_velocity_y != other.angular_velocity_y:
            return False
        if self.angular_velocity_z != other.angular_velocity_z:
            return False
        if self.failure_flags != other.failure_flags:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def target_system(self):
        """Message field 'target_system'."""
        return self._target_system

    @target_system.setter
    def target_system(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_system' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'target_system' field must be an unsigned integer in [0, 255]"
        self._target_system = value

    @builtins.property
    def target_component(self):
        """Message field 'target_component'."""
        return self._target_component

    @target_component.setter
    def target_component(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_component' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'target_component' field must be an unsigned integer in [0, 255]"
        self._target_component = value

    @builtins.property
    def flags(self):
        """Message field 'flags'."""
        return self._flags

    @flags.setter
    def flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'flags' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'flags' field must be an unsigned integer in [0, 65535]"
        self._flags = value

    @builtins.property
    def q(self):
        """Message field 'q'."""
        return self._q

    @q.setter
    def q(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'q' field must be a sub message of type 'Quaternion'"
        self._q = value

    @builtins.property
    def angular_velocity_x(self):
        """Message field 'angular_velocity_x'."""
        return self._angular_velocity_x

    @angular_velocity_x.setter
    def angular_velocity_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_velocity_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_velocity_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_velocity_x = value

    @builtins.property
    def angular_velocity_y(self):
        """Message field 'angular_velocity_y'."""
        return self._angular_velocity_y

    @angular_velocity_y.setter
    def angular_velocity_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_velocity_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_velocity_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_velocity_y = value

    @builtins.property
    def angular_velocity_z(self):
        """Message field 'angular_velocity_z'."""
        return self._angular_velocity_z

    @angular_velocity_z.setter
    def angular_velocity_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_velocity_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_velocity_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_velocity_z = value

    @builtins.property
    def failure_flags(self):
        """Message field 'failure_flags'."""
        return self._failure_flags

    @failure_flags.setter
    def failure_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'failure_flags' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'failure_flags' field must be an unsigned integer in [0, 4294967295]"
        self._failure_flags = value
