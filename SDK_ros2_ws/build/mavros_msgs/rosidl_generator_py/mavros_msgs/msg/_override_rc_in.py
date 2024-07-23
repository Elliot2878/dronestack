# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/OverrideRCIn.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'channels'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OverrideRCIn(type):
    """Metaclass of message 'OverrideRCIn'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CHAN_RELEASE': 0,
        'CHAN_NOCHANGE': 65535,
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
                'mavros_msgs.msg.OverrideRCIn')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__override_rc_in
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__override_rc_in
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__override_rc_in
            cls._TYPE_SUPPORT = module.type_support_msg__msg__override_rc_in
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__override_rc_in

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CHAN_RELEASE': cls.__constants['CHAN_RELEASE'],
            'CHAN_NOCHANGE': cls.__constants['CHAN_NOCHANGE'],
        }

    @property
    def CHAN_RELEASE(self):
        """Message constant 'CHAN_RELEASE'."""
        return Metaclass_OverrideRCIn.__constants['CHAN_RELEASE']

    @property
    def CHAN_NOCHANGE(self):
        """Message constant 'CHAN_NOCHANGE'."""
        return Metaclass_OverrideRCIn.__constants['CHAN_NOCHANGE']


class OverrideRCIn(metaclass=Metaclass_OverrideRCIn):
    """
    Message class 'OverrideRCIn'.

    Constants:
      CHAN_RELEASE
      CHAN_NOCHANGE
    """

    __slots__ = [
        '_channels',
    ]

    _fields_and_field_types = {
        'channels': 'uint16[18]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint16'), 18),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'channels' not in kwargs:
            self.channels = numpy.zeros(18, dtype=numpy.uint16)
        else:
            self.channels = numpy.array(kwargs.get('channels'), dtype=numpy.uint16)
            assert self.channels.shape == (18, )

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
        if all(self.channels != other.channels):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def channels(self):
        """Message field 'channels'."""
        return self._channels

    @channels.setter
    def channels(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint16, \
                "The 'channels' numpy.ndarray() must have the dtype of 'numpy.uint16'"
            assert value.size == 18, \
                "The 'channels' numpy.ndarray() must have a size of 18"
            self._channels = value
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
                 len(value) == 18 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'channels' field must be a set or sequence with length 18 and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._channels = numpy.array(value, dtype=numpy.uint16)
