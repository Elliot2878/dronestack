# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/TerrainReport.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TerrainReport(type):
    """Metaclass of message 'TerrainReport'."""

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
                'mavros_msgs.msg.TerrainReport')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__terrain_report
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__terrain_report
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__terrain_report
            cls._TYPE_SUPPORT = module.type_support_msg__msg__terrain_report
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__terrain_report

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


class TerrainReport(metaclass=Metaclass_TerrainReport):
    """Message class 'TerrainReport'."""

    __slots__ = [
        '_header',
        '_latitude',
        '_longitude',
        '_spacing',
        '_terrain_height',
        '_current_height',
        '_pending',
        '_loaded',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'latitude': 'double',
        'longitude': 'double',
        'spacing': 'uint16',
        'terrain_height': 'float',
        'current_height': 'float',
        'pending': 'uint16',
        'loaded': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.spacing = kwargs.get('spacing', int())
        self.terrain_height = kwargs.get('terrain_height', float())
        self.current_height = kwargs.get('current_height', float())
        self.pending = kwargs.get('pending', int())
        self.loaded = kwargs.get('loaded', int())

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
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.spacing != other.spacing:
            return False
        if self.terrain_height != other.terrain_height:
            return False
        if self.current_height != other.current_height:
            return False
        if self.pending != other.pending:
            return False
        if self.loaded != other.loaded:
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
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

    @builtins.property
    def spacing(self):
        """Message field 'spacing'."""
        return self._spacing

    @spacing.setter
    def spacing(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'spacing' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'spacing' field must be an unsigned integer in [0, 65535]"
        self._spacing = value

    @builtins.property
    def terrain_height(self):
        """Message field 'terrain_height'."""
        return self._terrain_height

    @terrain_height.setter
    def terrain_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'terrain_height' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'terrain_height' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._terrain_height = value

    @builtins.property
    def current_height(self):
        """Message field 'current_height'."""
        return self._current_height

    @current_height.setter
    def current_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_height' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_height' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_height = value

    @builtins.property
    def pending(self):
        """Message field 'pending'."""
        return self._pending

    @pending.setter
    def pending(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pending' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'pending' field must be an unsigned integer in [0, 65535]"
        self._pending = value

    @builtins.property
    def loaded(self):
        """Message field 'loaded'."""
        return self._loaded

    @loaded.setter
    def loaded(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'loaded' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'loaded' field must be an unsigned integer in [0, 65535]"
        self._loaded = value
