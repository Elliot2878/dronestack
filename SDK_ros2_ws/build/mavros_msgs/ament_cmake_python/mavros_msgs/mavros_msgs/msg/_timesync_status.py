# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/TimesyncStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TimesyncStatus(type):
    """Metaclass of message 'TimesyncStatus'."""

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
                'mavros_msgs.msg.TimesyncStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__timesync_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__timesync_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__timesync_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__timesync_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__timesync_status

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


class TimesyncStatus(metaclass=Metaclass_TimesyncStatus):
    """Message class 'TimesyncStatus'."""

    __slots__ = [
        '_header',
        '_remote_timestamp_ns',
        '_observed_offset_ns',
        '_estimated_offset_ns',
        '_round_trip_time_ms',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'remote_timestamp_ns': 'uint64',
        'observed_offset_ns': 'int64',
        'estimated_offset_ns': 'int64',
        'round_trip_time_ms': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.remote_timestamp_ns = kwargs.get('remote_timestamp_ns', int())
        self.observed_offset_ns = kwargs.get('observed_offset_ns', int())
        self.estimated_offset_ns = kwargs.get('estimated_offset_ns', int())
        self.round_trip_time_ms = kwargs.get('round_trip_time_ms', float())

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
        if self.remote_timestamp_ns != other.remote_timestamp_ns:
            return False
        if self.observed_offset_ns != other.observed_offset_ns:
            return False
        if self.estimated_offset_ns != other.estimated_offset_ns:
            return False
        if self.round_trip_time_ms != other.round_trip_time_ms:
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
    def remote_timestamp_ns(self):
        """Message field 'remote_timestamp_ns'."""
        return self._remote_timestamp_ns

    @remote_timestamp_ns.setter
    def remote_timestamp_ns(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'remote_timestamp_ns' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'remote_timestamp_ns' field must be an unsigned integer in [0, 18446744073709551615]"
        self._remote_timestamp_ns = value

    @builtins.property
    def observed_offset_ns(self):
        """Message field 'observed_offset_ns'."""
        return self._observed_offset_ns

    @observed_offset_ns.setter
    def observed_offset_ns(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'observed_offset_ns' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'observed_offset_ns' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._observed_offset_ns = value

    @builtins.property
    def estimated_offset_ns(self):
        """Message field 'estimated_offset_ns'."""
        return self._estimated_offset_ns

    @estimated_offset_ns.setter
    def estimated_offset_ns(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'estimated_offset_ns' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'estimated_offset_ns' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._estimated_offset_ns = value

    @builtins.property
    def round_trip_time_ms(self):
        """Message field 'round_trip_time_ms'."""
        return self._round_trip_time_ms

    @round_trip_time_ms.setter
    def round_trip_time_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'round_trip_time_ms' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'round_trip_time_ms' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._round_trip_time_ms = value
