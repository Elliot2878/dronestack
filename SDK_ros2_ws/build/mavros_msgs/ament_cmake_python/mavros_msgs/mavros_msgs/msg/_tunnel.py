# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/Tunnel.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'payload'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Tunnel(type):
    """Metaclass of message 'Tunnel'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PAYLOAD_TYPE_UNKNOWN': 0,
        'PAYLOAD_TYPE_STORM32_RESERVED0': 200,
        'PAYLOAD_TYPE_STORM32_RESERVED1': 201,
        'PAYLOAD_TYPE_STORM32_RESERVED2': 202,
        'PAYLOAD_TYPE_STORM32_RESERVED3': 203,
        'PAYLOAD_TYPE_STORM32_RESERVED4': 204,
        'PAYLOAD_TYPE_STORM32_RESERVED5': 205,
        'PAYLOAD_TYPE_STORM32_RESERVED6': 206,
        'PAYLOAD_TYPE_STORM32_RESERVED7': 207,
        'PAYLOAD_TYPE_STORM32_RESERVED8': 208,
        'PAYLOAD_TYPE_STORM32_RESERVED9': 209,
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
                'mavros_msgs.msg.Tunnel')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tunnel
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tunnel
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tunnel
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tunnel
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tunnel

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PAYLOAD_TYPE_UNKNOWN': cls.__constants['PAYLOAD_TYPE_UNKNOWN'],
            'PAYLOAD_TYPE_STORM32_RESERVED0': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED0'],
            'PAYLOAD_TYPE_STORM32_RESERVED1': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED1'],
            'PAYLOAD_TYPE_STORM32_RESERVED2': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED2'],
            'PAYLOAD_TYPE_STORM32_RESERVED3': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED3'],
            'PAYLOAD_TYPE_STORM32_RESERVED4': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED4'],
            'PAYLOAD_TYPE_STORM32_RESERVED5': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED5'],
            'PAYLOAD_TYPE_STORM32_RESERVED6': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED6'],
            'PAYLOAD_TYPE_STORM32_RESERVED7': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED7'],
            'PAYLOAD_TYPE_STORM32_RESERVED8': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED8'],
            'PAYLOAD_TYPE_STORM32_RESERVED9': cls.__constants['PAYLOAD_TYPE_STORM32_RESERVED9'],
        }

    @property
    def PAYLOAD_TYPE_UNKNOWN(self):
        """Message constant 'PAYLOAD_TYPE_UNKNOWN'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_UNKNOWN']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED0(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED0'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED0']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED1(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED1'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED1']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED2(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED2'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED2']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED3(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED3'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED3']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED4(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED4'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED4']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED5(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED5'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED5']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED6(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED6'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED6']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED7(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED7'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED7']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED8(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED8'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED8']

    @property
    def PAYLOAD_TYPE_STORM32_RESERVED9(self):
        """Message constant 'PAYLOAD_TYPE_STORM32_RESERVED9'."""
        return Metaclass_Tunnel.__constants['PAYLOAD_TYPE_STORM32_RESERVED9']


class Tunnel(metaclass=Metaclass_Tunnel):
    """
    Message class 'Tunnel'.

    Constants:
      PAYLOAD_TYPE_UNKNOWN
      PAYLOAD_TYPE_STORM32_RESERVED0
      PAYLOAD_TYPE_STORM32_RESERVED1
      PAYLOAD_TYPE_STORM32_RESERVED2
      PAYLOAD_TYPE_STORM32_RESERVED3
      PAYLOAD_TYPE_STORM32_RESERVED4
      PAYLOAD_TYPE_STORM32_RESERVED5
      PAYLOAD_TYPE_STORM32_RESERVED6
      PAYLOAD_TYPE_STORM32_RESERVED7
      PAYLOAD_TYPE_STORM32_RESERVED8
      PAYLOAD_TYPE_STORM32_RESERVED9
    """

    __slots__ = [
        '_target_system',
        '_target_component',
        '_payload_type',
        '_payload_length',
        '_payload',
    ]

    _fields_and_field_types = {
        'target_system': 'uint8',
        'target_component': 'uint8',
        'payload_type': 'uint16',
        'payload_length': 'uint8',
        'payload': 'uint8[128]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 128),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target_system = kwargs.get('target_system', int())
        self.target_component = kwargs.get('target_component', int())
        self.payload_type = kwargs.get('payload_type', int())
        self.payload_length = kwargs.get('payload_length', int())
        if 'payload' not in kwargs:
            self.payload = numpy.zeros(128, dtype=numpy.uint8)
        else:
            self.payload = numpy.array(kwargs.get('payload'), dtype=numpy.uint8)
            assert self.payload.shape == (128, )

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
        if self.target_system != other.target_system:
            return False
        if self.target_component != other.target_component:
            return False
        if self.payload_type != other.payload_type:
            return False
        if self.payload_length != other.payload_length:
            return False
        if all(self.payload != other.payload):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_system(self):
        """Message field 'target_system'."""
        return self._target_system

    @target_system.setter
    def target_system(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_system' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'target_system' field must be an unsigned integer in [0, 255]"
        self._target_system = value

    @builtins.property
    def target_component(self):
        """Message field 'target_component'."""
        return self._target_component

    @target_component.setter
    def target_component(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_component' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'target_component' field must be an unsigned integer in [0, 255]"
        self._target_component = value

    @builtins.property
    def payload_type(self):
        """Message field 'payload_type'."""
        return self._payload_type

    @payload_type.setter
    def payload_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'payload_type' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'payload_type' field must be an unsigned integer in [0, 65535]"
        self._payload_type = value

    @builtins.property
    def payload_length(self):
        """Message field 'payload_length'."""
        return self._payload_length

    @payload_length.setter
    def payload_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'payload_length' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'payload_length' field must be an unsigned integer in [0, 255]"
        self._payload_length = value

    @builtins.property
    def payload(self):
        """Message field 'payload'."""
        return self._payload

    @payload.setter
    def payload(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'payload' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 128, \
                "The 'payload' numpy.ndarray() must have a size of 128"
            self._payload = value
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
                 len(value) == 128 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'payload' field must be a set or sequence with length 128 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._payload = numpy.array(value, dtype=numpy.uint8)
