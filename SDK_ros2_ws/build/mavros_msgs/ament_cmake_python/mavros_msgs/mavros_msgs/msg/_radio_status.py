# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/RadioStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RadioStatus(type):
    """Metaclass of message 'RadioStatus'."""

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
                'mavros_msgs.msg.RadioStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__radio_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__radio_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__radio_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__radio_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__radio_status

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


class RadioStatus(metaclass=Metaclass_RadioStatus):
    """Message class 'RadioStatus'."""

    __slots__ = [
        '_header',
        '_rssi',
        '_remrssi',
        '_txbuf',
        '_noise',
        '_remnoise',
        '_rxerrors',
        '_fixed',
        '_rssi_dbm',
        '_remrssi_dbm',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'rssi': 'uint8',
        'remrssi': 'uint8',
        'txbuf': 'uint8',
        'noise': 'uint8',
        'remnoise': 'uint8',
        'rxerrors': 'uint16',
        'fixed': 'uint16',
        'rssi_dbm': 'float',
        'remrssi_dbm': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.rssi = kwargs.get('rssi', int())
        self.remrssi = kwargs.get('remrssi', int())
        self.txbuf = kwargs.get('txbuf', int())
        self.noise = kwargs.get('noise', int())
        self.remnoise = kwargs.get('remnoise', int())
        self.rxerrors = kwargs.get('rxerrors', int())
        self.fixed = kwargs.get('fixed', int())
        self.rssi_dbm = kwargs.get('rssi_dbm', float())
        self.remrssi_dbm = kwargs.get('remrssi_dbm', float())

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
        if self.rssi != other.rssi:
            return False
        if self.remrssi != other.remrssi:
            return False
        if self.txbuf != other.txbuf:
            return False
        if self.noise != other.noise:
            return False
        if self.remnoise != other.remnoise:
            return False
        if self.rxerrors != other.rxerrors:
            return False
        if self.fixed != other.fixed:
            return False
        if self.rssi_dbm != other.rssi_dbm:
            return False
        if self.remrssi_dbm != other.remrssi_dbm:
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
    def rssi(self):
        """Message field 'rssi'."""
        return self._rssi

    @rssi.setter
    def rssi(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rssi' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rssi' field must be an unsigned integer in [0, 255]"
        self._rssi = value

    @builtins.property
    def remrssi(self):
        """Message field 'remrssi'."""
        return self._remrssi

    @remrssi.setter
    def remrssi(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'remrssi' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'remrssi' field must be an unsigned integer in [0, 255]"
        self._remrssi = value

    @builtins.property
    def txbuf(self):
        """Message field 'txbuf'."""
        return self._txbuf

    @txbuf.setter
    def txbuf(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'txbuf' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'txbuf' field must be an unsigned integer in [0, 255]"
        self._txbuf = value

    @builtins.property
    def noise(self):
        """Message field 'noise'."""
        return self._noise

    @noise.setter
    def noise(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'noise' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'noise' field must be an unsigned integer in [0, 255]"
        self._noise = value

    @builtins.property
    def remnoise(self):
        """Message field 'remnoise'."""
        return self._remnoise

    @remnoise.setter
    def remnoise(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'remnoise' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'remnoise' field must be an unsigned integer in [0, 255]"
        self._remnoise = value

    @builtins.property
    def rxerrors(self):
        """Message field 'rxerrors'."""
        return self._rxerrors

    @rxerrors.setter
    def rxerrors(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rxerrors' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'rxerrors' field must be an unsigned integer in [0, 65535]"
        self._rxerrors = value

    @builtins.property
    def fixed(self):
        """Message field 'fixed'."""
        return self._fixed

    @fixed.setter
    def fixed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fixed' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'fixed' field must be an unsigned integer in [0, 65535]"
        self._fixed = value

    @builtins.property
    def rssi_dbm(self):
        """Message field 'rssi_dbm'."""
        return self._rssi_dbm

    @rssi_dbm.setter
    def rssi_dbm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rssi_dbm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rssi_dbm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rssi_dbm = value

    @builtins.property
    def remrssi_dbm(self):
        """Message field 'remrssi_dbm'."""
        return self._remrssi_dbm

    @remrssi_dbm.setter
    def remrssi_dbm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'remrssi_dbm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'remrssi_dbm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._remrssi_dbm = value
