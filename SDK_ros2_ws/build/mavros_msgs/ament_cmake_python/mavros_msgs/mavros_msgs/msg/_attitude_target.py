# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/AttitudeTarget.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AttitudeTarget(type):
    """Metaclass of message 'AttitudeTarget'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'IGNORE_ROLL_RATE': 1,
        'IGNORE_PITCH_RATE': 2,
        'IGNORE_YAW_RATE': 4,
        'IGNORE_THRUST': 64,
        'IGNORE_ATTITUDE': 128,
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
                'mavros_msgs.msg.AttitudeTarget')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__attitude_target
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__attitude_target
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__attitude_target
            cls._TYPE_SUPPORT = module.type_support_msg__msg__attitude_target
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__attitude_target

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

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
            'IGNORE_ROLL_RATE': cls.__constants['IGNORE_ROLL_RATE'],
            'IGNORE_PITCH_RATE': cls.__constants['IGNORE_PITCH_RATE'],
            'IGNORE_YAW_RATE': cls.__constants['IGNORE_YAW_RATE'],
            'IGNORE_THRUST': cls.__constants['IGNORE_THRUST'],
            'IGNORE_ATTITUDE': cls.__constants['IGNORE_ATTITUDE'],
        }

    @property
    def IGNORE_ROLL_RATE(self):
        """Message constant 'IGNORE_ROLL_RATE'."""
        return Metaclass_AttitudeTarget.__constants['IGNORE_ROLL_RATE']

    @property
    def IGNORE_PITCH_RATE(self):
        """Message constant 'IGNORE_PITCH_RATE'."""
        return Metaclass_AttitudeTarget.__constants['IGNORE_PITCH_RATE']

    @property
    def IGNORE_YAW_RATE(self):
        """Message constant 'IGNORE_YAW_RATE'."""
        return Metaclass_AttitudeTarget.__constants['IGNORE_YAW_RATE']

    @property
    def IGNORE_THRUST(self):
        """Message constant 'IGNORE_THRUST'."""
        return Metaclass_AttitudeTarget.__constants['IGNORE_THRUST']

    @property
    def IGNORE_ATTITUDE(self):
        """Message constant 'IGNORE_ATTITUDE'."""
        return Metaclass_AttitudeTarget.__constants['IGNORE_ATTITUDE']


class AttitudeTarget(metaclass=Metaclass_AttitudeTarget):
    """
    Message class 'AttitudeTarget'.

    Constants:
      IGNORE_ROLL_RATE
      IGNORE_PITCH_RATE
      IGNORE_YAW_RATE
      IGNORE_THRUST
      IGNORE_ATTITUDE
    """

    __slots__ = [
        '_header',
        '_type_mask',
        '_orientation',
        '_body_rate',
        '_thrust',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'type_mask': 'uint8',
        'orientation': 'geometry_msgs/Quaternion',
        'body_rate': 'geometry_msgs/Vector3',
        'thrust': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.type_mask = kwargs.get('type_mask', int())
        from geometry_msgs.msg import Quaternion
        self.orientation = kwargs.get('orientation', Quaternion())
        from geometry_msgs.msg import Vector3
        self.body_rate = kwargs.get('body_rate', Vector3())
        self.thrust = kwargs.get('thrust', float())

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
        if self.type_mask != other.type_mask:
            return False
        if self.orientation != other.orientation:
            return False
        if self.body_rate != other.body_rate:
            return False
        if self.thrust != other.thrust:
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
    def type_mask(self):
        """Message field 'type_mask'."""
        return self._type_mask

    @type_mask.setter
    def type_mask(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type_mask' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type_mask' field must be an unsigned integer in [0, 255]"
        self._type_mask = value

    @builtins.property
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'orientation' field must be a sub message of type 'Quaternion'"
        self._orientation = value

    @builtins.property
    def body_rate(self):
        """Message field 'body_rate'."""
        return self._body_rate

    @body_rate.setter
    def body_rate(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'body_rate' field must be a sub message of type 'Vector3'"
        self._body_rate = value

    @builtins.property
    def thrust(self):
        """Message field 'thrust'."""
        return self._thrust

    @thrust.setter
    def thrust(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thrust' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thrust = value
