# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/DebugValue.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'data'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DebugValue(type):
    """Metaclass of message 'DebugValue'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TYPE_DEBUG': 0,
        'TYPE_DEBUG_VECT': 1,
        'TYPE_DEBUG_FLOAT_ARRAY': 2,
        'TYPE_NAMED_VALUE_FLOAT': 3,
        'TYPE_NAMED_VALUE_INT': 4,
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
                'mavros_msgs.msg.DebugValue')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__debug_value
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__debug_value
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__debug_value
            cls._TYPE_SUPPORT = module.type_support_msg__msg__debug_value
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__debug_value

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TYPE_DEBUG': cls.__constants['TYPE_DEBUG'],
            'TYPE_DEBUG_VECT': cls.__constants['TYPE_DEBUG_VECT'],
            'TYPE_DEBUG_FLOAT_ARRAY': cls.__constants['TYPE_DEBUG_FLOAT_ARRAY'],
            'TYPE_NAMED_VALUE_FLOAT': cls.__constants['TYPE_NAMED_VALUE_FLOAT'],
            'TYPE_NAMED_VALUE_INT': cls.__constants['TYPE_NAMED_VALUE_INT'],
        }

    @property
    def TYPE_DEBUG(self):
        """Message constant 'TYPE_DEBUG'."""
        return Metaclass_DebugValue.__constants['TYPE_DEBUG']

    @property
    def TYPE_DEBUG_VECT(self):
        """Message constant 'TYPE_DEBUG_VECT'."""
        return Metaclass_DebugValue.__constants['TYPE_DEBUG_VECT']

    @property
    def TYPE_DEBUG_FLOAT_ARRAY(self):
        """Message constant 'TYPE_DEBUG_FLOAT_ARRAY'."""
        return Metaclass_DebugValue.__constants['TYPE_DEBUG_FLOAT_ARRAY']

    @property
    def TYPE_NAMED_VALUE_FLOAT(self):
        """Message constant 'TYPE_NAMED_VALUE_FLOAT'."""
        return Metaclass_DebugValue.__constants['TYPE_NAMED_VALUE_FLOAT']

    @property
    def TYPE_NAMED_VALUE_INT(self):
        """Message constant 'TYPE_NAMED_VALUE_INT'."""
        return Metaclass_DebugValue.__constants['TYPE_NAMED_VALUE_INT']


class DebugValue(metaclass=Metaclass_DebugValue):
    """
    Message class 'DebugValue'.

    Constants:
      TYPE_DEBUG
      TYPE_DEBUG_VECT
      TYPE_DEBUG_FLOAT_ARRAY
      TYPE_NAMED_VALUE_FLOAT
      TYPE_NAMED_VALUE_INT
    """

    __slots__ = [
        '_header',
        '_index',
        '_array_id',
        '_name',
        '_value_float',
        '_value_int',
        '_data',
        '_type',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'index': 'int32',
        'array_id': 'int32',
        'name': 'string',
        'value_float': 'float',
        'value_int': 'int32',
        'data': 'sequence<float>',
        'type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.index = kwargs.get('index', int())
        self.array_id = kwargs.get('array_id', int())
        self.name = kwargs.get('name', str())
        self.value_float = kwargs.get('value_float', float())
        self.value_int = kwargs.get('value_int', int())
        self.data = array.array('f', kwargs.get('data', []))
        self.type = kwargs.get('type', int())

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
        if self.index != other.index:
            return False
        if self.array_id != other.array_id:
            return False
        if self.name != other.name:
            return False
        if self.value_float != other.value_float:
            return False
        if self.value_int != other.value_int:
            return False
        if self.data != other.data:
            return False
        if self.type != other.type:
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
    def index(self):
        """Message field 'index'."""
        return self._index

    @index.setter
    def index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'index' field must be an integer in [-2147483648, 2147483647]"
        self._index = value

    @builtins.property
    def array_id(self):
        """Message field 'array_id'."""
        return self._array_id

    @array_id.setter
    def array_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'array_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'array_id' field must be an integer in [-2147483648, 2147483647]"
        self._array_id = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def value_float(self):
        """Message field 'value_float'."""
        return self._value_float

    @value_float.setter
    def value_float(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'value_float' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'value_float' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._value_float = value

    @builtins.property
    def value_int(self):
        """Message field 'value_int'."""
        return self._value_int

    @value_int.setter
    def value_int(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'value_int' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'value_int' field must be an integer in [-2147483648, 2147483647]"
        self._value_int = value

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'data' array.array() must have the type code of 'f'"
            self._data = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'data' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._data = array.array('f', value)

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value
