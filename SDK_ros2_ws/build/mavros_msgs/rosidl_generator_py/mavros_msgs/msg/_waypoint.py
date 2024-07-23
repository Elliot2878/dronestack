# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/Waypoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Waypoint(type):
    """Metaclass of message 'Waypoint'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FRAME_GLOBAL': 0,
        'FRAME_LOCAL_NED': 1,
        'FRAME_MISSION': 2,
        'FRAME_GLOBAL_REL_ALT': 3,
        'FRAME_LOCAL_ENU': 4,
        'FRAME_GLOBAL_INT': 5,
        'FRAME_GLOBAL_RELATIVE_ALT_INT': 6,
        'FRAME_LOCAL_OFFSET_NED': 7,
        'FRAME_BODY_NED': 8,
        'FRAME_BODY_OFFSET_NED': 9,
        'FRAME_GLOBAL_TERRAIN_ALT': 10,
        'FRAME_GLOBAL_TERRAIN_ALT_INT': 11,
        'FRAME_BODY_FRD': 12,
        'FRAME_RESERVED_13': 13,
        'FRAME_RESERVED_14': 14,
        'FRAME_RESERVED_15': 15,
        'FRAME_RESERVED_16': 16,
        'FRAME_RESERVED_17': 17,
        'FRAME_RESERVED_18': 18,
        'FRAME_RESERVED_19': 19,
        'FRAME_LOCAL_FRD': 20,
        'FRAME_LOCAL_FLU': 21,
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
                'mavros_msgs.msg.Waypoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__waypoint
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__waypoint
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__waypoint
            cls._TYPE_SUPPORT = module.type_support_msg__msg__waypoint
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__waypoint

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FRAME_GLOBAL': cls.__constants['FRAME_GLOBAL'],
            'FRAME_LOCAL_NED': cls.__constants['FRAME_LOCAL_NED'],
            'FRAME_MISSION': cls.__constants['FRAME_MISSION'],
            'FRAME_GLOBAL_REL_ALT': cls.__constants['FRAME_GLOBAL_REL_ALT'],
            'FRAME_LOCAL_ENU': cls.__constants['FRAME_LOCAL_ENU'],
            'FRAME_GLOBAL_INT': cls.__constants['FRAME_GLOBAL_INT'],
            'FRAME_GLOBAL_RELATIVE_ALT_INT': cls.__constants['FRAME_GLOBAL_RELATIVE_ALT_INT'],
            'FRAME_LOCAL_OFFSET_NED': cls.__constants['FRAME_LOCAL_OFFSET_NED'],
            'FRAME_BODY_NED': cls.__constants['FRAME_BODY_NED'],
            'FRAME_BODY_OFFSET_NED': cls.__constants['FRAME_BODY_OFFSET_NED'],
            'FRAME_GLOBAL_TERRAIN_ALT': cls.__constants['FRAME_GLOBAL_TERRAIN_ALT'],
            'FRAME_GLOBAL_TERRAIN_ALT_INT': cls.__constants['FRAME_GLOBAL_TERRAIN_ALT_INT'],
            'FRAME_BODY_FRD': cls.__constants['FRAME_BODY_FRD'],
            'FRAME_RESERVED_13': cls.__constants['FRAME_RESERVED_13'],
            'FRAME_RESERVED_14': cls.__constants['FRAME_RESERVED_14'],
            'FRAME_RESERVED_15': cls.__constants['FRAME_RESERVED_15'],
            'FRAME_RESERVED_16': cls.__constants['FRAME_RESERVED_16'],
            'FRAME_RESERVED_17': cls.__constants['FRAME_RESERVED_17'],
            'FRAME_RESERVED_18': cls.__constants['FRAME_RESERVED_18'],
            'FRAME_RESERVED_19': cls.__constants['FRAME_RESERVED_19'],
            'FRAME_LOCAL_FRD': cls.__constants['FRAME_LOCAL_FRD'],
            'FRAME_LOCAL_FLU': cls.__constants['FRAME_LOCAL_FLU'],
        }

    @property
    def FRAME_GLOBAL(self):
        """Message constant 'FRAME_GLOBAL'."""
        return Metaclass_Waypoint.__constants['FRAME_GLOBAL']

    @property
    def FRAME_LOCAL_NED(self):
        """Message constant 'FRAME_LOCAL_NED'."""
        return Metaclass_Waypoint.__constants['FRAME_LOCAL_NED']

    @property
    def FRAME_MISSION(self):
        """Message constant 'FRAME_MISSION'."""
        return Metaclass_Waypoint.__constants['FRAME_MISSION']

    @property
    def FRAME_GLOBAL_REL_ALT(self):
        """Message constant 'FRAME_GLOBAL_REL_ALT'."""
        return Metaclass_Waypoint.__constants['FRAME_GLOBAL_REL_ALT']

    @property
    def FRAME_LOCAL_ENU(self):
        """Message constant 'FRAME_LOCAL_ENU'."""
        return Metaclass_Waypoint.__constants['FRAME_LOCAL_ENU']

    @property
    def FRAME_GLOBAL_INT(self):
        """Message constant 'FRAME_GLOBAL_INT'."""
        return Metaclass_Waypoint.__constants['FRAME_GLOBAL_INT']

    @property
    def FRAME_GLOBAL_RELATIVE_ALT_INT(self):
        """Message constant 'FRAME_GLOBAL_RELATIVE_ALT_INT'."""
        return Metaclass_Waypoint.__constants['FRAME_GLOBAL_RELATIVE_ALT_INT']

    @property
    def FRAME_LOCAL_OFFSET_NED(self):
        """Message constant 'FRAME_LOCAL_OFFSET_NED'."""
        return Metaclass_Waypoint.__constants['FRAME_LOCAL_OFFSET_NED']

    @property
    def FRAME_BODY_NED(self):
        """Message constant 'FRAME_BODY_NED'."""
        return Metaclass_Waypoint.__constants['FRAME_BODY_NED']

    @property
    def FRAME_BODY_OFFSET_NED(self):
        """Message constant 'FRAME_BODY_OFFSET_NED'."""
        return Metaclass_Waypoint.__constants['FRAME_BODY_OFFSET_NED']

    @property
    def FRAME_GLOBAL_TERRAIN_ALT(self):
        """Message constant 'FRAME_GLOBAL_TERRAIN_ALT'."""
        return Metaclass_Waypoint.__constants['FRAME_GLOBAL_TERRAIN_ALT']

    @property
    def FRAME_GLOBAL_TERRAIN_ALT_INT(self):
        """Message constant 'FRAME_GLOBAL_TERRAIN_ALT_INT'."""
        return Metaclass_Waypoint.__constants['FRAME_GLOBAL_TERRAIN_ALT_INT']

    @property
    def FRAME_BODY_FRD(self):
        """Message constant 'FRAME_BODY_FRD'."""
        return Metaclass_Waypoint.__constants['FRAME_BODY_FRD']

    @property
    def FRAME_RESERVED_13(self):
        """Message constant 'FRAME_RESERVED_13'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_13']

    @property
    def FRAME_RESERVED_14(self):
        """Message constant 'FRAME_RESERVED_14'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_14']

    @property
    def FRAME_RESERVED_15(self):
        """Message constant 'FRAME_RESERVED_15'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_15']

    @property
    def FRAME_RESERVED_16(self):
        """Message constant 'FRAME_RESERVED_16'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_16']

    @property
    def FRAME_RESERVED_17(self):
        """Message constant 'FRAME_RESERVED_17'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_17']

    @property
    def FRAME_RESERVED_18(self):
        """Message constant 'FRAME_RESERVED_18'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_18']

    @property
    def FRAME_RESERVED_19(self):
        """Message constant 'FRAME_RESERVED_19'."""
        return Metaclass_Waypoint.__constants['FRAME_RESERVED_19']

    @property
    def FRAME_LOCAL_FRD(self):
        """Message constant 'FRAME_LOCAL_FRD'."""
        return Metaclass_Waypoint.__constants['FRAME_LOCAL_FRD']

    @property
    def FRAME_LOCAL_FLU(self):
        """Message constant 'FRAME_LOCAL_FLU'."""
        return Metaclass_Waypoint.__constants['FRAME_LOCAL_FLU']


class Waypoint(metaclass=Metaclass_Waypoint):
    """
    Message class 'Waypoint'.

    Constants:
      FRAME_GLOBAL
      FRAME_LOCAL_NED
      FRAME_MISSION
      FRAME_GLOBAL_REL_ALT
      FRAME_LOCAL_ENU
      FRAME_GLOBAL_INT
      FRAME_GLOBAL_RELATIVE_ALT_INT
      FRAME_LOCAL_OFFSET_NED
      FRAME_BODY_NED
      FRAME_BODY_OFFSET_NED
      FRAME_GLOBAL_TERRAIN_ALT
      FRAME_GLOBAL_TERRAIN_ALT_INT
      FRAME_BODY_FRD
      FRAME_RESERVED_13
      FRAME_RESERVED_14
      FRAME_RESERVED_15
      FRAME_RESERVED_16
      FRAME_RESERVED_17
      FRAME_RESERVED_18
      FRAME_RESERVED_19
      FRAME_LOCAL_FRD
      FRAME_LOCAL_FLU
    """

    __slots__ = [
        '_frame',
        '_command',
        '_is_current',
        '_autocontinue',
        '_param1',
        '_param2',
        '_param3',
        '_param4',
        '_x_lat',
        '_y_long',
        '_z_alt',
    ]

    _fields_and_field_types = {
        'frame': 'uint8',
        'command': 'uint16',
        'is_current': 'boolean',
        'autocontinue': 'boolean',
        'param1': 'float',
        'param2': 'float',
        'param3': 'float',
        'param4': 'float',
        'x_lat': 'double',
        'y_long': 'double',
        'z_alt': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.frame = kwargs.get('frame', int())
        self.command = kwargs.get('command', int())
        self.is_current = kwargs.get('is_current', bool())
        self.autocontinue = kwargs.get('autocontinue', bool())
        self.param1 = kwargs.get('param1', float())
        self.param2 = kwargs.get('param2', float())
        self.param3 = kwargs.get('param3', float())
        self.param4 = kwargs.get('param4', float())
        self.x_lat = kwargs.get('x_lat', float())
        self.y_long = kwargs.get('y_long', float())
        self.z_alt = kwargs.get('z_alt', float())

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
        if self.frame != other.frame:
            return False
        if self.command != other.command:
            return False
        if self.is_current != other.is_current:
            return False
        if self.autocontinue != other.autocontinue:
            return False
        if self.param1 != other.param1:
            return False
        if self.param2 != other.param2:
            return False
        if self.param3 != other.param3:
            return False
        if self.param4 != other.param4:
            return False
        if self.x_lat != other.x_lat:
            return False
        if self.y_long != other.y_long:
            return False
        if self.z_alt != other.z_alt:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def frame(self):
        """Message field 'frame'."""
        return self._frame

    @frame.setter
    def frame(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frame' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'frame' field must be an unsigned integer in [0, 255]"
        self._frame = value

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'command' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'command' field must be an unsigned integer in [0, 65535]"
        self._command = value

    @builtins.property
    def is_current(self):
        """Message field 'is_current'."""
        return self._is_current

    @is_current.setter
    def is_current(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_current' field must be of type 'bool'"
        self._is_current = value

    @builtins.property
    def autocontinue(self):
        """Message field 'autocontinue'."""
        return self._autocontinue

    @autocontinue.setter
    def autocontinue(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'autocontinue' field must be of type 'bool'"
        self._autocontinue = value

    @builtins.property
    def param1(self):
        """Message field 'param1'."""
        return self._param1

    @param1.setter
    def param1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'param1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'param1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._param1 = value

    @builtins.property
    def param2(self):
        """Message field 'param2'."""
        return self._param2

    @param2.setter
    def param2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'param2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'param2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._param2 = value

    @builtins.property
    def param3(self):
        """Message field 'param3'."""
        return self._param3

    @param3.setter
    def param3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'param3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'param3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._param3 = value

    @builtins.property
    def param4(self):
        """Message field 'param4'."""
        return self._param4

    @param4.setter
    def param4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'param4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'param4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._param4 = value

    @builtins.property
    def x_lat(self):
        """Message field 'x_lat'."""
        return self._x_lat

    @x_lat.setter
    def x_lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_lat = value

    @builtins.property
    def y_long(self):
        """Message field 'y_long'."""
        return self._y_long

    @y_long.setter
    def y_long(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_long' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_long' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_long = value

    @builtins.property
    def z_alt(self):
        """Message field 'z_alt'."""
        return self._z_alt

    @z_alt.setter
    def z_alt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_alt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z_alt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z_alt = value
