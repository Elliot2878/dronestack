# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/Mavlink.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'payload64'
# Member 'signature'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Mavlink(type):
    """Metaclass of message 'Mavlink'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FRAMING_OK': 1,
        'FRAMING_BAD_CRC': 2,
        'FRAMING_BAD_SIGNATURE': 3,
        'MAVLINK_V10': 254,
        'MAVLINK_V20': 253,
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
                'mavros_msgs.msg.Mavlink')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mavlink
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mavlink
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mavlink
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mavlink
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mavlink

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FRAMING_OK': cls.__constants['FRAMING_OK'],
            'FRAMING_BAD_CRC': cls.__constants['FRAMING_BAD_CRC'],
            'FRAMING_BAD_SIGNATURE': cls.__constants['FRAMING_BAD_SIGNATURE'],
            'MAVLINK_V10': cls.__constants['MAVLINK_V10'],
            'MAVLINK_V20': cls.__constants['MAVLINK_V20'],
        }

    @property
    def FRAMING_OK(self):
        """Message constant 'FRAMING_OK'."""
        return Metaclass_Mavlink.__constants['FRAMING_OK']

    @property
    def FRAMING_BAD_CRC(self):
        """Message constant 'FRAMING_BAD_CRC'."""
        return Metaclass_Mavlink.__constants['FRAMING_BAD_CRC']

    @property
    def FRAMING_BAD_SIGNATURE(self):
        """Message constant 'FRAMING_BAD_SIGNATURE'."""
        return Metaclass_Mavlink.__constants['FRAMING_BAD_SIGNATURE']

    @property
    def MAVLINK_V10(self):
        """Message constant 'MAVLINK_V10'."""
        return Metaclass_Mavlink.__constants['MAVLINK_V10']

    @property
    def MAVLINK_V20(self):
        """Message constant 'MAVLINK_V20'."""
        return Metaclass_Mavlink.__constants['MAVLINK_V20']


class Mavlink(metaclass=Metaclass_Mavlink):
    """
    Message class 'Mavlink'.

    Constants:
      FRAMING_OK
      FRAMING_BAD_CRC
      FRAMING_BAD_SIGNATURE
      MAVLINK_V10
      MAVLINK_V20
    """

    __slots__ = [
        '_header',
        '_framing_status',
        '_magic',
        '_len',
        '_incompat_flags',
        '_compat_flags',
        '_seq',
        '_sysid',
        '_compid',
        '_msgid',
        '_checksum',
        '_payload64',
        '_signature',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'framing_status': 'uint8',
        'magic': 'uint8',
        'len': 'uint8',
        'incompat_flags': 'uint8',
        'compat_flags': 'uint8',
        'seq': 'uint8',
        'sysid': 'uint8',
        'compid': 'uint8',
        'msgid': 'uint32',
        'checksum': 'uint16',
        'payload64': 'sequence<uint64, 33>',
        'signature': 'sequence<uint8, 13>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('uint64'), 33),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('uint8'), 13),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.framing_status = kwargs.get('framing_status', int())
        self.magic = kwargs.get('magic', int())
        self.len = kwargs.get('len', int())
        self.incompat_flags = kwargs.get('incompat_flags', int())
        self.compat_flags = kwargs.get('compat_flags', int())
        self.seq = kwargs.get('seq', int())
        self.sysid = kwargs.get('sysid', int())
        self.compid = kwargs.get('compid', int())
        self.msgid = kwargs.get('msgid', int())
        self.checksum = kwargs.get('checksum', int())
        self.payload64 = array.array('Q', kwargs.get('payload64', []))
        self.signature = array.array('B', kwargs.get('signature', []))

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
        if self.framing_status != other.framing_status:
            return False
        if self.magic != other.magic:
            return False
        if self.len != other.len:
            return False
        if self.incompat_flags != other.incompat_flags:
            return False
        if self.compat_flags != other.compat_flags:
            return False
        if self.seq != other.seq:
            return False
        if self.sysid != other.sysid:
            return False
        if self.compid != other.compid:
            return False
        if self.msgid != other.msgid:
            return False
        if self.checksum != other.checksum:
            return False
        if self.payload64 != other.payload64:
            return False
        if self.signature != other.signature:
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
    def framing_status(self):
        """Message field 'framing_status'."""
        return self._framing_status

    @framing_status.setter
    def framing_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'framing_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'framing_status' field must be an unsigned integer in [0, 255]"
        self._framing_status = value

    @builtins.property
    def magic(self):
        """Message field 'magic'."""
        return self._magic

    @magic.setter
    def magic(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'magic' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'magic' field must be an unsigned integer in [0, 255]"
        self._magic = value

    @builtins.property  # noqa: A003
    def len(self):  # noqa: A003
        """Message field 'len'."""
        return self._len

    @len.setter  # noqa: A003
    def len(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'len' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'len' field must be an unsigned integer in [0, 255]"
        self._len = value

    @builtins.property
    def incompat_flags(self):
        """Message field 'incompat_flags'."""
        return self._incompat_flags

    @incompat_flags.setter
    def incompat_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'incompat_flags' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'incompat_flags' field must be an unsigned integer in [0, 255]"
        self._incompat_flags = value

    @builtins.property
    def compat_flags(self):
        """Message field 'compat_flags'."""
        return self._compat_flags

    @compat_flags.setter
    def compat_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compat_flags' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compat_flags' field must be an unsigned integer in [0, 255]"
        self._compat_flags = value

    @builtins.property
    def seq(self):
        """Message field 'seq'."""
        return self._seq

    @seq.setter
    def seq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seq' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'seq' field must be an unsigned integer in [0, 255]"
        self._seq = value

    @builtins.property
    def sysid(self):
        """Message field 'sysid'."""
        return self._sysid

    @sysid.setter
    def sysid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sysid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sysid' field must be an unsigned integer in [0, 255]"
        self._sysid = value

    @builtins.property
    def compid(self):
        """Message field 'compid'."""
        return self._compid

    @compid.setter
    def compid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compid' field must be an unsigned integer in [0, 255]"
        self._compid = value

    @builtins.property
    def msgid(self):
        """Message field 'msgid'."""
        return self._msgid

    @msgid.setter
    def msgid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'msgid' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'msgid' field must be an unsigned integer in [0, 4294967295]"
        self._msgid = value

    @builtins.property
    def checksum(self):
        """Message field 'checksum'."""
        return self._checksum

    @checksum.setter
    def checksum(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'checksum' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'checksum' field must be an unsigned integer in [0, 65535]"
        self._checksum = value

    @builtins.property
    def payload64(self):
        """Message field 'payload64'."""
        return self._payload64

    @payload64.setter
    def payload64(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'Q', \
                "The 'payload64' array.array() must have the type code of 'Q'"
            assert len(value) <= 33, \
                "The 'payload64' array.array() must have a size <= 33"
            self._payload64 = value
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
                 len(value) <= 33 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 18446744073709551616 for val in value)), \
                "The 'payload64' field must be a set or sequence with length <= 33 and each value of type 'int' and each unsigned integer in [0, 18446744073709551615]"
        self._payload64 = array.array('Q', value)

    @builtins.property
    def signature(self):
        """Message field 'signature'."""
        return self._signature

    @signature.setter
    def signature(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'signature' array.array() must have the type code of 'B'"
            assert len(value) <= 13, \
                "The 'signature' array.array() must have a size <= 13"
            self._signature = value
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
                 len(value) <= 13 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'signature' field must be a set or sequence with length <= 13 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._signature = array.array('B', value)
