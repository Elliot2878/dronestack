# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/NavControllerOutput.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavControllerOutput(type):
    """Metaclass of message 'NavControllerOutput'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'mavros_msgs.msg.NavControllerOutput')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__nav_controller_output
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__nav_controller_output
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__nav_controller_output
            cls._TYPE_SUPPORT = module.type_support_msg__msg__nav_controller_output
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__nav_controller_output

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavControllerOutput(metaclass=Metaclass_NavControllerOutput):
    """Message class 'NavControllerOutput'."""

    __slots__ = [
        '_header',
        '_nav_roll',
        '_nav_pitch',
        '_nav_bearing',
        '_target_bearing',
        '_wp_dist',
        '_alt_error',
        '_aspd_error',
        '_xtrack_error',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'nav_roll': 'float',
        'nav_pitch': 'float',
        'nav_bearing': 'int16',
        'target_bearing': 'int16',
        'wp_dist': 'uint16',
        'alt_error': 'float',
        'aspd_error': 'float',
        'xtrack_error': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
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
        self.nav_roll = kwargs.get('nav_roll', float())
        self.nav_pitch = kwargs.get('nav_pitch', float())
        self.nav_bearing = kwargs.get('nav_bearing', int())
        self.target_bearing = kwargs.get('target_bearing', int())
        self.wp_dist = kwargs.get('wp_dist', int())
        self.alt_error = kwargs.get('alt_error', float())
        self.aspd_error = kwargs.get('aspd_error', float())
        self.xtrack_error = kwargs.get('xtrack_error', float())

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
        if self.nav_roll != other.nav_roll:
            return False
        if self.nav_pitch != other.nav_pitch:
            return False
        if self.nav_bearing != other.nav_bearing:
            return False
        if self.target_bearing != other.target_bearing:
            return False
        if self.wp_dist != other.wp_dist:
            return False
        if self.alt_error != other.alt_error:
            return False
        if self.aspd_error != other.aspd_error:
            return False
        if self.xtrack_error != other.xtrack_error:
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
    def nav_roll(self):
        """Message field 'nav_roll'."""
        return self._nav_roll

    @nav_roll.setter
    def nav_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nav_roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'nav_roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._nav_roll = value

    @builtins.property
    def nav_pitch(self):
        """Message field 'nav_pitch'."""
        return self._nav_pitch

    @nav_pitch.setter
    def nav_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nav_pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'nav_pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._nav_pitch = value

    @builtins.property
    def nav_bearing(self):
        """Message field 'nav_bearing'."""
        return self._nav_bearing

    @nav_bearing.setter
    def nav_bearing(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'nav_bearing' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'nav_bearing' field must be an integer in [-32768, 32767]"
        self._nav_bearing = value

    @builtins.property
    def target_bearing(self):
        """Message field 'target_bearing'."""
        return self._target_bearing

    @target_bearing.setter
    def target_bearing(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_bearing' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'target_bearing' field must be an integer in [-32768, 32767]"
        self._target_bearing = value

    @builtins.property
    def wp_dist(self):
        """Message field 'wp_dist'."""
        return self._wp_dist

    @wp_dist.setter
    def wp_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'wp_dist' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'wp_dist' field must be an unsigned integer in [0, 65535]"
        self._wp_dist = value

    @builtins.property
    def alt_error(self):
        """Message field 'alt_error'."""
        return self._alt_error

    @alt_error.setter
    def alt_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'alt_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'alt_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._alt_error = value

    @builtins.property
    def aspd_error(self):
        """Message field 'aspd_error'."""
        return self._aspd_error

    @aspd_error.setter
    def aspd_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aspd_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aspd_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aspd_error = value

    @builtins.property
    def xtrack_error(self):
        """Message field 'xtrack_error'."""
        return self._xtrack_error

    @xtrack_error.setter
    def xtrack_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'xtrack_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'xtrack_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._xtrack_error = value
