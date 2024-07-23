# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/GPSRTK.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GPSRTK(type):
    """Metaclass of message 'GPSRTK'."""

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
                'mavros_msgs.msg.GPSRTK')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gpsrtk
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gpsrtk
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gpsrtk
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gpsrtk
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gpsrtk

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


class GPSRTK(metaclass=Metaclass_GPSRTK):
    """Message class 'GPSRTK'."""

    __slots__ = [
        '_header',
        '_rtk_receiver_id',
        '_wn',
        '_tow',
        '_rtk_health',
        '_rtk_rate',
        '_nsats',
        '_baseline_a',
        '_baseline_b',
        '_baseline_c',
        '_accuracy',
        '_iar_num_hypotheses',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'rtk_receiver_id': 'uint8',
        'wn': 'int16',
        'tow': 'uint32',
        'rtk_health': 'uint8',
        'rtk_rate': 'uint8',
        'nsats': 'uint8',
        'baseline_a': 'int32',
        'baseline_b': 'int32',
        'baseline_c': 'int32',
        'accuracy': 'uint32',
        'iar_num_hypotheses': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.rtk_receiver_id = kwargs.get('rtk_receiver_id', int())
        self.wn = kwargs.get('wn', int())
        self.tow = kwargs.get('tow', int())
        self.rtk_health = kwargs.get('rtk_health', int())
        self.rtk_rate = kwargs.get('rtk_rate', int())
        self.nsats = kwargs.get('nsats', int())
        self.baseline_a = kwargs.get('baseline_a', int())
        self.baseline_b = kwargs.get('baseline_b', int())
        self.baseline_c = kwargs.get('baseline_c', int())
        self.accuracy = kwargs.get('accuracy', int())
        self.iar_num_hypotheses = kwargs.get('iar_num_hypotheses', int())

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
        if self.rtk_receiver_id != other.rtk_receiver_id:
            return False
        if self.wn != other.wn:
            return False
        if self.tow != other.tow:
            return False
        if self.rtk_health != other.rtk_health:
            return False
        if self.rtk_rate != other.rtk_rate:
            return False
        if self.nsats != other.nsats:
            return False
        if self.baseline_a != other.baseline_a:
            return False
        if self.baseline_b != other.baseline_b:
            return False
        if self.baseline_c != other.baseline_c:
            return False
        if self.accuracy != other.accuracy:
            return False
        if self.iar_num_hypotheses != other.iar_num_hypotheses:
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
    def rtk_receiver_id(self):
        """Message field 'rtk_receiver_id'."""
        return self._rtk_receiver_id

    @rtk_receiver_id.setter
    def rtk_receiver_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rtk_receiver_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rtk_receiver_id' field must be an unsigned integer in [0, 255]"
        self._rtk_receiver_id = value

    @builtins.property
    def wn(self):
        """Message field 'wn'."""
        return self._wn

    @wn.setter
    def wn(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'wn' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'wn' field must be an integer in [-32768, 32767]"
        self._wn = value

    @builtins.property
    def tow(self):
        """Message field 'tow'."""
        return self._tow

    @tow.setter
    def tow(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tow' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'tow' field must be an unsigned integer in [0, 4294967295]"
        self._tow = value

    @builtins.property
    def rtk_health(self):
        """Message field 'rtk_health'."""
        return self._rtk_health

    @rtk_health.setter
    def rtk_health(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rtk_health' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rtk_health' field must be an unsigned integer in [0, 255]"
        self._rtk_health = value

    @builtins.property
    def rtk_rate(self):
        """Message field 'rtk_rate'."""
        return self._rtk_rate

    @rtk_rate.setter
    def rtk_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rtk_rate' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rtk_rate' field must be an unsigned integer in [0, 255]"
        self._rtk_rate = value

    @builtins.property
    def nsats(self):
        """Message field 'nsats'."""
        return self._nsats

    @nsats.setter
    def nsats(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'nsats' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'nsats' field must be an unsigned integer in [0, 255]"
        self._nsats = value

    @builtins.property
    def baseline_a(self):
        """Message field 'baseline_a'."""
        return self._baseline_a

    @baseline_a.setter
    def baseline_a(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'baseline_a' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'baseline_a' field must be an integer in [-2147483648, 2147483647]"
        self._baseline_a = value

    @builtins.property
    def baseline_b(self):
        """Message field 'baseline_b'."""
        return self._baseline_b

    @baseline_b.setter
    def baseline_b(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'baseline_b' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'baseline_b' field must be an integer in [-2147483648, 2147483647]"
        self._baseline_b = value

    @builtins.property
    def baseline_c(self):
        """Message field 'baseline_c'."""
        return self._baseline_c

    @baseline_c.setter
    def baseline_c(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'baseline_c' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'baseline_c' field must be an integer in [-2147483648, 2147483647]"
        self._baseline_c = value

    @builtins.property
    def accuracy(self):
        """Message field 'accuracy'."""
        return self._accuracy

    @accuracy.setter
    def accuracy(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'accuracy' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'accuracy' field must be an unsigned integer in [0, 4294967295]"
        self._accuracy = value

    @builtins.property
    def iar_num_hypotheses(self):
        """Message field 'iar_num_hypotheses'."""
        return self._iar_num_hypotheses

    @iar_num_hypotheses.setter
    def iar_num_hypotheses(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'iar_num_hypotheses' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'iar_num_hypotheses' field must be an integer in [-2147483648, 2147483647]"
        self._iar_num_hypotheses = value
