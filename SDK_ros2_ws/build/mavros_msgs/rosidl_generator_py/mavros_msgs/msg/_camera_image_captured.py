# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/CameraImageCaptured.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CameraImageCaptured(type):
    """Metaclass of message 'CameraImageCaptured'."""

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
                'mavros_msgs.msg.CameraImageCaptured')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__camera_image_captured
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__camera_image_captured
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__camera_image_captured
            cls._TYPE_SUPPORT = module.type_support_msg__msg__camera_image_captured
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__camera_image_captured

            from geographic_msgs.msg import GeoPoint
            if GeoPoint.__class__._TYPE_SUPPORT is None:
                GeoPoint.__class__.__import_type_support__()

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

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


class CameraImageCaptured(metaclass=Metaclass_CameraImageCaptured):
    """Message class 'CameraImageCaptured'."""

    __slots__ = [
        '_header',
        '_orientation',
        '_geo',
        '_relative_alt',
        '_image_index',
        '_capture_result',
        '_file_url',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'orientation': 'geometry_msgs/Quaternion',
        'geo': 'geographic_msgs/GeoPoint',
        'relative_alt': 'float',
        'image_index': 'int32',
        'capture_result': 'int8',
        'file_url': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geographic_msgs', 'msg'], 'GeoPoint'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Quaternion
        self.orientation = kwargs.get('orientation', Quaternion())
        from geographic_msgs.msg import GeoPoint
        self.geo = kwargs.get('geo', GeoPoint())
        self.relative_alt = kwargs.get('relative_alt', float())
        self.image_index = kwargs.get('image_index', int())
        self.capture_result = kwargs.get('capture_result', int())
        self.file_url = kwargs.get('file_url', str())

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
        if self.orientation != other.orientation:
            return False
        if self.geo != other.geo:
            return False
        if self.relative_alt != other.relative_alt:
            return False
        if self.image_index != other.image_index:
            return False
        if self.capture_result != other.capture_result:
            return False
        if self.file_url != other.file_url:
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
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'orientation' field must be a sub message of type 'Quaternion'"
        self._orientation = value

    @builtins.property
    def geo(self):
        """Message field 'geo'."""
        return self._geo

    @geo.setter
    def geo(self, value):
        if __debug__:
            from geographic_msgs.msg import GeoPoint
            assert \
                isinstance(value, GeoPoint), \
                "The 'geo' field must be a sub message of type 'GeoPoint'"
        self._geo = value

    @builtins.property
    def relative_alt(self):
        """Message field 'relative_alt'."""
        return self._relative_alt

    @relative_alt.setter
    def relative_alt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'relative_alt' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'relative_alt' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._relative_alt = value

    @builtins.property
    def image_index(self):
        """Message field 'image_index'."""
        return self._image_index

    @image_index.setter
    def image_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'image_index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'image_index' field must be an integer in [-2147483648, 2147483647]"
        self._image_index = value

    @builtins.property
    def capture_result(self):
        """Message field 'capture_result'."""
        return self._capture_result

    @capture_result.setter
    def capture_result(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'capture_result' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'capture_result' field must be an integer in [-128, 127]"
        self._capture_result = value

    @builtins.property
    def file_url(self):
        """Message field 'file_url'."""
        return self._file_url

    @file_url.setter
    def file_url(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'file_url' field must be of type 'str'"
        self._file_url = value
