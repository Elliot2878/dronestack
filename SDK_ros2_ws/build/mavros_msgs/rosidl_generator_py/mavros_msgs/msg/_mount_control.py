# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/MountControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MountControl(type):
    """Metaclass of message 'MountControl'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAV_MOUNT_MODE_RETRACT': 0,
        'MAV_MOUNT_MODE_NEUTRAL': 1,
        'MAV_MOUNT_MODE_MAVLINK_TARGETING': 2,
        'MAV_MOUNT_MODE_RC_TARGETING': 3,
        'MAV_MOUNT_MODE_GPS_POINT': 4,
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
                'mavros_msgs.msg.MountControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mount_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mount_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mount_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mount_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mount_control

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAV_MOUNT_MODE_RETRACT': cls.__constants['MAV_MOUNT_MODE_RETRACT'],
            'MAV_MOUNT_MODE_NEUTRAL': cls.__constants['MAV_MOUNT_MODE_NEUTRAL'],
            'MAV_MOUNT_MODE_MAVLINK_TARGETING': cls.__constants['MAV_MOUNT_MODE_MAVLINK_TARGETING'],
            'MAV_MOUNT_MODE_RC_TARGETING': cls.__constants['MAV_MOUNT_MODE_RC_TARGETING'],
            'MAV_MOUNT_MODE_GPS_POINT': cls.__constants['MAV_MOUNT_MODE_GPS_POINT'],
        }

    @property
    def MAV_MOUNT_MODE_RETRACT(self):
        """Message constant 'MAV_MOUNT_MODE_RETRACT'."""
        return Metaclass_MountControl.__constants['MAV_MOUNT_MODE_RETRACT']

    @property
    def MAV_MOUNT_MODE_NEUTRAL(self):
        """Message constant 'MAV_MOUNT_MODE_NEUTRAL'."""
        return Metaclass_MountControl.__constants['MAV_MOUNT_MODE_NEUTRAL']

    @property
    def MAV_MOUNT_MODE_MAVLINK_TARGETING(self):
        """Message constant 'MAV_MOUNT_MODE_MAVLINK_TARGETING'."""
        return Metaclass_MountControl.__constants['MAV_MOUNT_MODE_MAVLINK_TARGETING']

    @property
    def MAV_MOUNT_MODE_RC_TARGETING(self):
        """Message constant 'MAV_MOUNT_MODE_RC_TARGETING'."""
        return Metaclass_MountControl.__constants['MAV_MOUNT_MODE_RC_TARGETING']

    @property
    def MAV_MOUNT_MODE_GPS_POINT(self):
        """Message constant 'MAV_MOUNT_MODE_GPS_POINT'."""
        return Metaclass_MountControl.__constants['MAV_MOUNT_MODE_GPS_POINT']


class MountControl(metaclass=Metaclass_MountControl):
    """
    Message class 'MountControl'.

    Constants:
      MAV_MOUNT_MODE_RETRACT
      MAV_MOUNT_MODE_NEUTRAL
      MAV_MOUNT_MODE_MAVLINK_TARGETING
      MAV_MOUNT_MODE_RC_TARGETING
      MAV_MOUNT_MODE_GPS_POINT
    """

    __slots__ = [
        '_header',
        '_mode',
        '_pitch',
        '_roll',
        '_yaw',
        '_altitude',
        '_latitude',
        '_longitude',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'mode': 'uint8',
        'pitch': 'float',
        'roll': 'float',
        'yaw': 'float',
        'altitude': 'float',
        'latitude': 'float',
        'longitude': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.mode = kwargs.get('mode', int())
        self.pitch = kwargs.get('pitch', float())
        self.roll = kwargs.get('roll', float())
        self.yaw = kwargs.get('yaw', float())
        self.altitude = kwargs.get('altitude', float())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())

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
        if self.mode != other.mode:
            return False
        if self.pitch != other.pitch:
            return False
        if self.roll != other.roll:
            return False
        if self.yaw != other.yaw:
            return False
        if self.altitude != other.altitude:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
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
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch = value

    @builtins.property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'altitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._altitude = value

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'latitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'longitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._longitude = value
