# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/LogEntry.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LogEntry(type):
    """Metaclass of message 'LogEntry'."""

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
                'mavros_msgs.msg.LogEntry')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__log_entry
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__log_entry
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__log_entry
            cls._TYPE_SUPPORT = module.type_support_msg__msg__log_entry
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__log_entry

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

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


class LogEntry(metaclass=Metaclass_LogEntry):
    """Message class 'LogEntry'."""

    __slots__ = [
        '_header',
        '_id',
        '_num_logs',
        '_last_log_num',
        '_time_utc',
        '_size',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'uint16',
        'num_logs': 'uint16',
        'last_log_num': 'uint16',
        'time_utc': 'builtin_interfaces/Time',
        'size': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', int())
        self.num_logs = kwargs.get('num_logs', int())
        self.last_log_num = kwargs.get('last_log_num', int())
        from builtin_interfaces.msg import Time
        self.time_utc = kwargs.get('time_utc', Time())
        self.size = kwargs.get('size', int())

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
        if self.id != other.id:
            return False
        if self.num_logs != other.num_logs:
            return False
        if self.last_log_num != other.last_log_num:
            return False
        if self.time_utc != other.time_utc:
            return False
        if self.size != other.size:
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

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'id' field must be an unsigned integer in [0, 65535]"
        self._id = value

    @builtins.property
    def num_logs(self):
        """Message field 'num_logs'."""
        return self._num_logs

    @num_logs.setter
    def num_logs(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_logs' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'num_logs' field must be an unsigned integer in [0, 65535]"
        self._num_logs = value

    @builtins.property
    def last_log_num(self):
        """Message field 'last_log_num'."""
        return self._last_log_num

    @last_log_num.setter
    def last_log_num(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'last_log_num' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'last_log_num' field must be an unsigned integer in [0, 65535]"
        self._last_log_num = value

    @builtins.property
    def time_utc(self):
        """Message field 'time_utc'."""
        return self._time_utc

    @time_utc.setter
    def time_utc(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'time_utc' field must be a sub message of type 'Time'"
        self._time_utc = value

    @builtins.property
    def size(self):
        """Message field 'size'."""
        return self._size

    @size.setter
    def size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'size' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'size' field must be an unsigned integer in [0, 4294967295]"
        self._size = value
