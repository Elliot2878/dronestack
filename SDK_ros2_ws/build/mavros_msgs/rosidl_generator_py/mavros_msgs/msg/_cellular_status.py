# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/CellularStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CellularStatus(type):
    """Metaclass of message 'CellularStatus'."""

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
                'mavros_msgs.msg.CellularStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cellular_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cellular_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cellular_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cellular_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cellular_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CellularStatus(metaclass=Metaclass_CellularStatus):
    """Message class 'CellularStatus'."""

    __slots__ = [
        '_status',
        '_failure_reason',
        '_type',
        '_quality',
        '_mcc',
        '_mnc',
        '_lac',
    ]

    _fields_and_field_types = {
        'status': 'uint8',
        'failure_reason': 'uint8',
        'type': 'uint8',
        'quality': 'uint8',
        'mcc': 'uint16',
        'mnc': 'uint16',
        'lac': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        self.failure_reason = kwargs.get('failure_reason', int())
        self.type = kwargs.get('type', int())
        self.quality = kwargs.get('quality', int())
        self.mcc = kwargs.get('mcc', int())
        self.mnc = kwargs.get('mnc', int())
        self.lac = kwargs.get('lac', int())

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
        if self.status != other.status:
            return False
        if self.failure_reason != other.failure_reason:
            return False
        if self.type != other.type:
            return False
        if self.quality != other.quality:
            return False
        if self.mcc != other.mcc:
            return False
        if self.mnc != other.mnc:
            return False
        if self.lac != other.lac:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value

    @builtins.property
    def failure_reason(self):
        """Message field 'failure_reason'."""
        return self._failure_reason

    @failure_reason.setter
    def failure_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'failure_reason' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'failure_reason' field must be an unsigned integer in [0, 255]"
        self._failure_reason = value

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

    @builtins.property
    def quality(self):
        """Message field 'quality'."""
        return self._quality

    @quality.setter
    def quality(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quality' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'quality' field must be an unsigned integer in [0, 255]"
        self._quality = value

    @builtins.property
    def mcc(self):
        """Message field 'mcc'."""
        return self._mcc

    @mcc.setter
    def mcc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mcc' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'mcc' field must be an unsigned integer in [0, 65535]"
        self._mcc = value

    @builtins.property
    def mnc(self):
        """Message field 'mnc'."""
        return self._mnc

    @mnc.setter
    def mnc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mnc' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'mnc' field must be an unsigned integer in [0, 65535]"
        self._mnc = value

    @builtins.property
    def lac(self):
        """Message field 'lac'."""
        return self._lac

    @lac.setter
    def lac(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lac' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'lac' field must be an unsigned integer in [0, 65535]"
        self._lac = value
