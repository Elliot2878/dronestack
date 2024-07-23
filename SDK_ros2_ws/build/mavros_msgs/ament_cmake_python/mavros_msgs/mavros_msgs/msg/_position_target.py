# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/PositionTarget.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PositionTarget(type):
    """Metaclass of message 'PositionTarget'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FRAME_LOCAL_NED': 1,
        'FRAME_LOCAL_OFFSET_NED': 7,
        'FRAME_BODY_NED': 8,
        'FRAME_BODY_OFFSET_NED': 9,
        'IGNORE_PX': 1,
        'IGNORE_PY': 2,
        'IGNORE_PZ': 4,
        'IGNORE_VX': 8,
        'IGNORE_VY': 16,
        'IGNORE_VZ': 32,
        'IGNORE_AFX': 64,
        'IGNORE_AFY': 128,
        'IGNORE_AFZ': 256,
        'FORCE': 512,
        'IGNORE_YAW': 1024,
        'IGNORE_YAW_RATE': 2048,
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
                'mavros_msgs.msg.PositionTarget')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__position_target
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__position_target
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__position_target
            cls._TYPE_SUPPORT = module.type_support_msg__msg__position_target
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__position_target

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FRAME_LOCAL_NED': cls.__constants['FRAME_LOCAL_NED'],
            'FRAME_LOCAL_OFFSET_NED': cls.__constants['FRAME_LOCAL_OFFSET_NED'],
            'FRAME_BODY_NED': cls.__constants['FRAME_BODY_NED'],
            'FRAME_BODY_OFFSET_NED': cls.__constants['FRAME_BODY_OFFSET_NED'],
            'IGNORE_PX': cls.__constants['IGNORE_PX'],
            'IGNORE_PY': cls.__constants['IGNORE_PY'],
            'IGNORE_PZ': cls.__constants['IGNORE_PZ'],
            'IGNORE_VX': cls.__constants['IGNORE_VX'],
            'IGNORE_VY': cls.__constants['IGNORE_VY'],
            'IGNORE_VZ': cls.__constants['IGNORE_VZ'],
            'IGNORE_AFX': cls.__constants['IGNORE_AFX'],
            'IGNORE_AFY': cls.__constants['IGNORE_AFY'],
            'IGNORE_AFZ': cls.__constants['IGNORE_AFZ'],
            'FORCE': cls.__constants['FORCE'],
            'IGNORE_YAW': cls.__constants['IGNORE_YAW'],
            'IGNORE_YAW_RATE': cls.__constants['IGNORE_YAW_RATE'],
        }

    @property
    def FRAME_LOCAL_NED(self):
        """Message constant 'FRAME_LOCAL_NED'."""
        return Metaclass_PositionTarget.__constants['FRAME_LOCAL_NED']

    @property
    def FRAME_LOCAL_OFFSET_NED(self):
        """Message constant 'FRAME_LOCAL_OFFSET_NED'."""
        return Metaclass_PositionTarget.__constants['FRAME_LOCAL_OFFSET_NED']

    @property
    def FRAME_BODY_NED(self):
        """Message constant 'FRAME_BODY_NED'."""
        return Metaclass_PositionTarget.__constants['FRAME_BODY_NED']

    @property
    def FRAME_BODY_OFFSET_NED(self):
        """Message constant 'FRAME_BODY_OFFSET_NED'."""
        return Metaclass_PositionTarget.__constants['FRAME_BODY_OFFSET_NED']

    @property
    def IGNORE_PX(self):
        """Message constant 'IGNORE_PX'."""
        return Metaclass_PositionTarget.__constants['IGNORE_PX']

    @property
    def IGNORE_PY(self):
        """Message constant 'IGNORE_PY'."""
        return Metaclass_PositionTarget.__constants['IGNORE_PY']

    @property
    def IGNORE_PZ(self):
        """Message constant 'IGNORE_PZ'."""
        return Metaclass_PositionTarget.__constants['IGNORE_PZ']

    @property
    def IGNORE_VX(self):
        """Message constant 'IGNORE_VX'."""
        return Metaclass_PositionTarget.__constants['IGNORE_VX']

    @property
    def IGNORE_VY(self):
        """Message constant 'IGNORE_VY'."""
        return Metaclass_PositionTarget.__constants['IGNORE_VY']

    @property
    def IGNORE_VZ(self):
        """Message constant 'IGNORE_VZ'."""
        return Metaclass_PositionTarget.__constants['IGNORE_VZ']

    @property
    def IGNORE_AFX(self):
        """Message constant 'IGNORE_AFX'."""
        return Metaclass_PositionTarget.__constants['IGNORE_AFX']

    @property
    def IGNORE_AFY(self):
        """Message constant 'IGNORE_AFY'."""
        return Metaclass_PositionTarget.__constants['IGNORE_AFY']

    @property
    def IGNORE_AFZ(self):
        """Message constant 'IGNORE_AFZ'."""
        return Metaclass_PositionTarget.__constants['IGNORE_AFZ']

    @property
    def FORCE(self):
        """Message constant 'FORCE'."""
        return Metaclass_PositionTarget.__constants['FORCE']

    @property
    def IGNORE_YAW(self):
        """Message constant 'IGNORE_YAW'."""
        return Metaclass_PositionTarget.__constants['IGNORE_YAW']

    @property
    def IGNORE_YAW_RATE(self):
        """Message constant 'IGNORE_YAW_RATE'."""
        return Metaclass_PositionTarget.__constants['IGNORE_YAW_RATE']


class PositionTarget(metaclass=Metaclass_PositionTarget):
    """
    Message class 'PositionTarget'.

    Constants:
      FRAME_LOCAL_NED
      FRAME_LOCAL_OFFSET_NED
      FRAME_BODY_NED
      FRAME_BODY_OFFSET_NED
      IGNORE_PX
      IGNORE_PY
      IGNORE_PZ
      IGNORE_VX
      IGNORE_VY
      IGNORE_VZ
      IGNORE_AFX
      IGNORE_AFY
      IGNORE_AFZ
      FORCE
      IGNORE_YAW
      IGNORE_YAW_RATE
    """

    __slots__ = [
        '_header',
        '_coordinate_frame',
        '_type_mask',
        '_position',
        '_velocity',
        '_acceleration_or_force',
        '_yaw',
        '_yaw_rate',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'coordinate_frame': 'uint8',
        'type_mask': 'uint16',
        'position': 'geometry_msgs/Point',
        'velocity': 'geometry_msgs/Vector3',
        'acceleration_or_force': 'geometry_msgs/Vector3',
        'yaw': 'float',
        'yaw_rate': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.coordinate_frame = kwargs.get('coordinate_frame', int())
        self.type_mask = kwargs.get('type_mask', int())
        from geometry_msgs.msg import Point
        self.position = kwargs.get('position', Point())
        from geometry_msgs.msg import Vector3
        self.velocity = kwargs.get('velocity', Vector3())
        from geometry_msgs.msg import Vector3
        self.acceleration_or_force = kwargs.get('acceleration_or_force', Vector3())
        self.yaw = kwargs.get('yaw', float())
        self.yaw_rate = kwargs.get('yaw_rate', float())

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
        if self.coordinate_frame != other.coordinate_frame:
            return False
        if self.type_mask != other.type_mask:
            return False
        if self.position != other.position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.acceleration_or_force != other.acceleration_or_force:
            return False
        if self.yaw != other.yaw:
            return False
        if self.yaw_rate != other.yaw_rate:
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
    def coordinate_frame(self):
        """Message field 'coordinate_frame'."""
        return self._coordinate_frame

    @coordinate_frame.setter
    def coordinate_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'coordinate_frame' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'coordinate_frame' field must be an unsigned integer in [0, 255]"
        self._coordinate_frame = value

    @builtins.property
    def type_mask(self):
        """Message field 'type_mask'."""
        return self._type_mask

    @type_mask.setter
    def type_mask(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type_mask' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'type_mask' field must be an unsigned integer in [0, 65535]"
        self._type_mask = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position' field must be a sub message of type 'Point'"
        self._position = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'velocity' field must be a sub message of type 'Vector3'"
        self._velocity = value

    @builtins.property
    def acceleration_or_force(self):
        """Message field 'acceleration_or_force'."""
        return self._acceleration_or_force

    @acceleration_or_force.setter
    def acceleration_or_force(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'acceleration_or_force' field must be a sub message of type 'Vector3'"
        self._acceleration_or_force = value

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
    def yaw_rate(self):
        """Message field 'yaw_rate'."""
        return self._yaw_rate

    @yaw_rate.setter
    def yaw_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_rate = value
