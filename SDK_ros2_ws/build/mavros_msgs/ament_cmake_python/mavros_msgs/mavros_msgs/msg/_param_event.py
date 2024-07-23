# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/ParamEvent.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ParamEvent(type):
    """Metaclass of message 'ParamEvent'."""

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
                'mavros_msgs.msg.ParamEvent')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__param_event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__param_event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__param_event
            cls._TYPE_SUPPORT = module.type_support_msg__msg__param_event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__param_event

            from rcl_interfaces.msg import ParameterValue
            if ParameterValue.__class__._TYPE_SUPPORT is None:
                ParameterValue.__class__.__import_type_support__()

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


class ParamEvent(metaclass=Metaclass_ParamEvent):
    """Message class 'ParamEvent'."""

    __slots__ = [
        '_header',
        '_param_id',
        '_value',
        '_param_index',
        '_param_count',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'param_id': 'string',
        'value': 'rcl_interfaces/ParameterValue',
        'param_index': 'uint16',
        'param_count': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rcl_interfaces', 'msg'], 'ParameterValue'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.param_id = kwargs.get('param_id', str())
        from rcl_interfaces.msg import ParameterValue
        self.value = kwargs.get('value', ParameterValue())
        self.param_index = kwargs.get('param_index', int())
        self.param_count = kwargs.get('param_count', int())

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
        if self.param_id != other.param_id:
            return False
        if self.value != other.value:
            return False
        if self.param_index != other.param_index:
            return False
        if self.param_count != other.param_count:
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
    def param_id(self):
        """Message field 'param_id'."""
        return self._param_id

    @param_id.setter
    def param_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'param_id' field must be of type 'str'"
        self._param_id = value

    @builtins.property
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if __debug__:
            from rcl_interfaces.msg import ParameterValue
            assert \
                isinstance(value, ParameterValue), \
                "The 'value' field must be a sub message of type 'ParameterValue'"
        self._value = value

    @builtins.property
    def param_index(self):
        """Message field 'param_index'."""
        return self._param_index

    @param_index.setter
    def param_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param_index' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'param_index' field must be an unsigned integer in [0, 65535]"
        self._param_index = value

    @builtins.property
    def param_count(self):
        """Message field 'param_count'."""
        return self._param_count

    @param_count.setter
    def param_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param_count' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'param_count' field must be an unsigned integer in [0, 65535]"
        self._param_count = value
