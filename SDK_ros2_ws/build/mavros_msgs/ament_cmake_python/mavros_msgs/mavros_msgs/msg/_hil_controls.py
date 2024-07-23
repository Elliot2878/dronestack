# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/HilControls.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HilControls(type):
    """Metaclass of message 'HilControls'."""

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
                'mavros_msgs.msg.HilControls')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hil_controls
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hil_controls
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hil_controls
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hil_controls
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hil_controls

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


class HilControls(metaclass=Metaclass_HilControls):
    """Message class 'HilControls'."""

    __slots__ = [
        '_header',
        '_roll_ailerons',
        '_pitch_elevator',
        '_yaw_rudder',
        '_throttle',
        '_aux1',
        '_aux2',
        '_aux3',
        '_aux4',
        '_mode',
        '_nav_mode',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'roll_ailerons': 'float',
        'pitch_elevator': 'float',
        'yaw_rudder': 'float',
        'throttle': 'float',
        'aux1': 'float',
        'aux2': 'float',
        'aux3': 'float',
        'aux4': 'float',
        'mode': 'uint8',
        'nav_mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.roll_ailerons = kwargs.get('roll_ailerons', float())
        self.pitch_elevator = kwargs.get('pitch_elevator', float())
        self.yaw_rudder = kwargs.get('yaw_rudder', float())
        self.throttle = kwargs.get('throttle', float())
        self.aux1 = kwargs.get('aux1', float())
        self.aux2 = kwargs.get('aux2', float())
        self.aux3 = kwargs.get('aux3', float())
        self.aux4 = kwargs.get('aux4', float())
        self.mode = kwargs.get('mode', int())
        self.nav_mode = kwargs.get('nav_mode', int())

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
        if self.roll_ailerons != other.roll_ailerons:
            return False
        if self.pitch_elevator != other.pitch_elevator:
            return False
        if self.yaw_rudder != other.yaw_rudder:
            return False
        if self.throttle != other.throttle:
            return False
        if self.aux1 != other.aux1:
            return False
        if self.aux2 != other.aux2:
            return False
        if self.aux3 != other.aux3:
            return False
        if self.aux4 != other.aux4:
            return False
        if self.mode != other.mode:
            return False
        if self.nav_mode != other.nav_mode:
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
    def roll_ailerons(self):
        """Message field 'roll_ailerons'."""
        return self._roll_ailerons

    @roll_ailerons.setter
    def roll_ailerons(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_ailerons' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_ailerons' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_ailerons = value

    @builtins.property
    def pitch_elevator(self):
        """Message field 'pitch_elevator'."""
        return self._pitch_elevator

    @pitch_elevator.setter
    def pitch_elevator(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_elevator' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_elevator' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_elevator = value

    @builtins.property
    def yaw_rudder(self):
        """Message field 'yaw_rudder'."""
        return self._yaw_rudder

    @yaw_rudder.setter
    def yaw_rudder(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_rudder' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_rudder' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_rudder = value

    @builtins.property
    def throttle(self):
        """Message field 'throttle'."""
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'throttle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._throttle = value

    @builtins.property
    def aux1(self):
        """Message field 'aux1'."""
        return self._aux1

    @aux1.setter
    def aux1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux1 = value

    @builtins.property
    def aux2(self):
        """Message field 'aux2'."""
        return self._aux2

    @aux2.setter
    def aux2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux2 = value

    @builtins.property
    def aux3(self):
        """Message field 'aux3'."""
        return self._aux3

    @aux3.setter
    def aux3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux3 = value

    @builtins.property
    def aux4(self):
        """Message field 'aux4'."""
        return self._aux4

    @aux4.setter
    def aux4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux4 = value

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
    def nav_mode(self):
        """Message field 'nav_mode'."""
        return self._nav_mode

    @nav_mode.setter
    def nav_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'nav_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'nav_mode' field must be an unsigned integer in [0, 255]"
        self._nav_mode = value
