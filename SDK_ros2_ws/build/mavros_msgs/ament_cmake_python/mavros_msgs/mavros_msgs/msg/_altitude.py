# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/Altitude.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Altitude(type):
    """Metaclass of message 'Altitude'."""

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
                'mavros_msgs.msg.Altitude')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__altitude
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__altitude
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__altitude
            cls._TYPE_SUPPORT = module.type_support_msg__msg__altitude
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__altitude

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


class Altitude(metaclass=Metaclass_Altitude):
    """Message class 'Altitude'."""

    __slots__ = [
        '_header',
        '_monotonic',
        '_amsl',
        '_local',
        '_relative',
        '_terrain',
        '_bottom_clearance',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'monotonic': 'float',
        'amsl': 'float',
        'local': 'float',
        'relative': 'float',
        'terrain': 'float',
        'bottom_clearance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.monotonic = kwargs.get('monotonic', float())
        self.amsl = kwargs.get('amsl', float())
        self.local = kwargs.get('local', float())
        self.relative = kwargs.get('relative', float())
        self.terrain = kwargs.get('terrain', float())
        self.bottom_clearance = kwargs.get('bottom_clearance', float())

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
        if self.monotonic != other.monotonic:
            return False
        if self.amsl != other.amsl:
            return False
        if self.local != other.local:
            return False
        if self.relative != other.relative:
            return False
        if self.terrain != other.terrain:
            return False
        if self.bottom_clearance != other.bottom_clearance:
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
    def monotonic(self):
        """Message field 'monotonic'."""
        return self._monotonic

    @monotonic.setter
    def monotonic(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'monotonic' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'monotonic' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._monotonic = value

    @builtins.property
    def amsl(self):
        """Message field 'amsl'."""
        return self._amsl

    @amsl.setter
    def amsl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'amsl' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'amsl' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._amsl = value

    @builtins.property
    def local(self):
        """Message field 'local'."""
        return self._local

    @local.setter
    def local(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'local' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'local' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._local = value

    @builtins.property
    def relative(self):
        """Message field 'relative'."""
        return self._relative

    @relative.setter
    def relative(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'relative' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'relative' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._relative = value

    @builtins.property
    def terrain(self):
        """Message field 'terrain'."""
        return self._terrain

    @terrain.setter
    def terrain(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'terrain' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'terrain' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._terrain = value

    @builtins.property
    def bottom_clearance(self):
        """Message field 'bottom_clearance'."""
        return self._bottom_clearance

    @bottom_clearance.setter
    def bottom_clearance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bottom_clearance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bottom_clearance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bottom_clearance = value
