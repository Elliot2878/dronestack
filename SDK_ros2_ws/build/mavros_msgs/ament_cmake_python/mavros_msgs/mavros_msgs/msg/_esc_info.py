# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/ESCInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ESCInfo(type):
    """Metaclass of message 'ESCInfo'."""

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
                'mavros_msgs.msg.ESCInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__esc_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__esc_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__esc_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__esc_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__esc_info

            from mavros_msgs.msg import ESCInfoItem
            if ESCInfoItem.__class__._TYPE_SUPPORT is None:
                ESCInfoItem.__class__.__import_type_support__()

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


class ESCInfo(metaclass=Metaclass_ESCInfo):
    """Message class 'ESCInfo'."""

    __slots__ = [
        '_header',
        '_counter',
        '_count',
        '_connection_type',
        '_info',
        '_esc_info',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'counter': 'uint16',
        'count': 'uint8',
        'connection_type': 'uint8',
        'info': 'uint8',
        'esc_info': 'sequence<mavros_msgs/ESCInfoItem>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'ESCInfoItem')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.counter = kwargs.get('counter', int())
        self.count = kwargs.get('count', int())
        self.connection_type = kwargs.get('connection_type', int())
        self.info = kwargs.get('info', int())
        self.esc_info = kwargs.get('esc_info', [])

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
        if self.counter != other.counter:
            return False
        if self.count != other.count:
            return False
        if self.connection_type != other.connection_type:
            return False
        if self.info != other.info:
            return False
        if self.esc_info != other.esc_info:
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
    def counter(self):
        """Message field 'counter'."""
        return self._counter

    @counter.setter
    def counter(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'counter' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'counter' field must be an unsigned integer in [0, 65535]"
        self._counter = value

    @builtins.property
    def count(self):
        """Message field 'count'."""
        return self._count

    @count.setter
    def count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'count' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'count' field must be an unsigned integer in [0, 255]"
        self._count = value

    @builtins.property
    def connection_type(self):
        """Message field 'connection_type'."""
        return self._connection_type

    @connection_type.setter
    def connection_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'connection_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'connection_type' field must be an unsigned integer in [0, 255]"
        self._connection_type = value

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'info' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'info' field must be an unsigned integer in [0, 255]"
        self._info = value

    @builtins.property
    def esc_info(self):
        """Message field 'esc_info'."""
        return self._esc_info

    @esc_info.setter
    def esc_info(self, value):
        if __debug__:
            from mavros_msgs.msg import ESCInfoItem
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, ESCInfoItem) for v in value) and
                 True), \
                "The 'esc_info' field must be a set or sequence and each value of type 'ESCInfoItem'"
        self._esc_info = value
