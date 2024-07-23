# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/GPSINPUT.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GPSINPUT(type):
    """Metaclass of message 'GPSINPUT'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GPS_FIX_TYPE_NO_GPS': 0,
        'GPS_FIX_TYPE_NO_FIX': 1,
        'GPS_FIX_TYPE_2D_FIX': 2,
        'GPS_FIX_TYPE_3D_FIX': 3,
        'GPS_FIX_TYPE_DGPS': 4,
        'GPS_FIX_TYPE_RTK_FLOATR': 5,
        'GPS_FIX_TYPE_RTK_FIXEDR': 6,
        'GPS_FIX_TYPE_STATIC': 7,
        'GPS_FIX_TYPE_PPP': 8,
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
                'mavros_msgs.msg.GPSINPUT')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gpsinput
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gpsinput
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gpsinput
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gpsinput
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gpsinput

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GPS_FIX_TYPE_NO_GPS': cls.__constants['GPS_FIX_TYPE_NO_GPS'],
            'GPS_FIX_TYPE_NO_FIX': cls.__constants['GPS_FIX_TYPE_NO_FIX'],
            'GPS_FIX_TYPE_2D_FIX': cls.__constants['GPS_FIX_TYPE_2D_FIX'],
            'GPS_FIX_TYPE_3D_FIX': cls.__constants['GPS_FIX_TYPE_3D_FIX'],
            'GPS_FIX_TYPE_DGPS': cls.__constants['GPS_FIX_TYPE_DGPS'],
            'GPS_FIX_TYPE_RTK_FLOATR': cls.__constants['GPS_FIX_TYPE_RTK_FLOATR'],
            'GPS_FIX_TYPE_RTK_FIXEDR': cls.__constants['GPS_FIX_TYPE_RTK_FIXEDR'],
            'GPS_FIX_TYPE_STATIC': cls.__constants['GPS_FIX_TYPE_STATIC'],
            'GPS_FIX_TYPE_PPP': cls.__constants['GPS_FIX_TYPE_PPP'],
        }

    @property
    def GPS_FIX_TYPE_NO_GPS(self):
        """Message constant 'GPS_FIX_TYPE_NO_GPS'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_NO_GPS']

    @property
    def GPS_FIX_TYPE_NO_FIX(self):
        """Message constant 'GPS_FIX_TYPE_NO_FIX'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_NO_FIX']

    @property
    def GPS_FIX_TYPE_2D_FIX(self):
        """Message constant 'GPS_FIX_TYPE_2D_FIX'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_2D_FIX']

    @property
    def GPS_FIX_TYPE_3D_FIX(self):
        """Message constant 'GPS_FIX_TYPE_3D_FIX'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_3D_FIX']

    @property
    def GPS_FIX_TYPE_DGPS(self):
        """Message constant 'GPS_FIX_TYPE_DGPS'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_DGPS']

    @property
    def GPS_FIX_TYPE_RTK_FLOATR(self):
        """Message constant 'GPS_FIX_TYPE_RTK_FLOATR'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_RTK_FLOATR']

    @property
    def GPS_FIX_TYPE_RTK_FIXEDR(self):
        """Message constant 'GPS_FIX_TYPE_RTK_FIXEDR'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_RTK_FIXEDR']

    @property
    def GPS_FIX_TYPE_STATIC(self):
        """Message constant 'GPS_FIX_TYPE_STATIC'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_STATIC']

    @property
    def GPS_FIX_TYPE_PPP(self):
        """Message constant 'GPS_FIX_TYPE_PPP'."""
        return Metaclass_GPSINPUT.__constants['GPS_FIX_TYPE_PPP']


class GPSINPUT(metaclass=Metaclass_GPSINPUT):
    """
    Message class 'GPSINPUT'.

    Constants:
      GPS_FIX_TYPE_NO_GPS
      GPS_FIX_TYPE_NO_FIX
      GPS_FIX_TYPE_2D_FIX
      GPS_FIX_TYPE_3D_FIX
      GPS_FIX_TYPE_DGPS
      GPS_FIX_TYPE_RTK_FLOATR
      GPS_FIX_TYPE_RTK_FIXEDR
      GPS_FIX_TYPE_STATIC
      GPS_FIX_TYPE_PPP
    """

    __slots__ = [
        '_header',
        '_fix_type',
        '_gps_id',
        '_ignore_flags',
        '_time_week_ms',
        '_time_week',
        '_lat',
        '_lon',
        '_alt',
        '_hdop',
        '_vdop',
        '_vn',
        '_ve',
        '_vd',
        '_speed_accuracy',
        '_horiz_accuracy',
        '_vert_accuracy',
        '_satellites_visible',
        '_yaw',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'fix_type': 'uint8',
        'gps_id': 'uint8',
        'ignore_flags': 'uint16',
        'time_week_ms': 'uint32',
        'time_week': 'uint16',
        'lat': 'int32',
        'lon': 'int32',
        'alt': 'float',
        'hdop': 'float',
        'vdop': 'float',
        'vn': 'float',
        've': 'float',
        'vd': 'float',
        'speed_accuracy': 'float',
        'horiz_accuracy': 'float',
        'vert_accuracy': 'float',
        'satellites_visible': 'uint8',
        'yaw': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.fix_type = kwargs.get('fix_type', int())
        self.gps_id = kwargs.get('gps_id', int())
        self.ignore_flags = kwargs.get('ignore_flags', int())
        self.time_week_ms = kwargs.get('time_week_ms', int())
        self.time_week = kwargs.get('time_week', int())
        self.lat = kwargs.get('lat', int())
        self.lon = kwargs.get('lon', int())
        self.alt = kwargs.get('alt', float())
        self.hdop = kwargs.get('hdop', float())
        self.vdop = kwargs.get('vdop', float())
        self.vn = kwargs.get('vn', float())
        self.ve = kwargs.get('ve', float())
        self.vd = kwargs.get('vd', float())
        self.speed_accuracy = kwargs.get('speed_accuracy', float())
        self.horiz_accuracy = kwargs.get('horiz_accuracy', float())
        self.vert_accuracy = kwargs.get('vert_accuracy', float())
        self.satellites_visible = kwargs.get('satellites_visible', int())
        self.yaw = kwargs.get('yaw', int())

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
        if self.fix_type != other.fix_type:
            return False
        if self.gps_id != other.gps_id:
            return False
        if self.ignore_flags != other.ignore_flags:
            return False
        if self.time_week_ms != other.time_week_ms:
            return False
        if self.time_week != other.time_week:
            return False
        if self.lat != other.lat:
            return False
        if self.lon != other.lon:
            return False
        if self.alt != other.alt:
            return False
        if self.hdop != other.hdop:
            return False
        if self.vdop != other.vdop:
            return False
        if self.vn != other.vn:
            return False
        if self.ve != other.ve:
            return False
        if self.vd != other.vd:
            return False
        if self.speed_accuracy != other.speed_accuracy:
            return False
        if self.horiz_accuracy != other.horiz_accuracy:
            return False
        if self.vert_accuracy != other.vert_accuracy:
            return False
        if self.satellites_visible != other.satellites_visible:
            return False
        if self.yaw != other.yaw:
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
    def fix_type(self):
        """Message field 'fix_type'."""
        return self._fix_type

    @fix_type.setter
    def fix_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fix_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fix_type' field must be an unsigned integer in [0, 255]"
        self._fix_type = value

    @builtins.property
    def gps_id(self):
        """Message field 'gps_id'."""
        return self._gps_id

    @gps_id.setter
    def gps_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gps_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gps_id' field must be an unsigned integer in [0, 255]"
        self._gps_id = value

    @builtins.property
    def ignore_flags(self):
        """Message field 'ignore_flags'."""
        return self._ignore_flags

    @ignore_flags.setter
    def ignore_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ignore_flags' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'ignore_flags' field must be an unsigned integer in [0, 65535]"
        self._ignore_flags = value

    @builtins.property
    def time_week_ms(self):
        """Message field 'time_week_ms'."""
        return self._time_week_ms

    @time_week_ms.setter
    def time_week_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_week_ms' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_week_ms' field must be an unsigned integer in [0, 4294967295]"
        self._time_week_ms = value

    @builtins.property
    def time_week(self):
        """Message field 'time_week'."""
        return self._time_week

    @time_week.setter
    def time_week(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_week' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'time_week' field must be an unsigned integer in [0, 65535]"
        self._time_week = value

    @builtins.property
    def lat(self):
        """Message field 'lat'."""
        return self._lat

    @lat.setter
    def lat(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lat' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'lat' field must be an integer in [-2147483648, 2147483647]"
        self._lat = value

    @builtins.property
    def lon(self):
        """Message field 'lon'."""
        return self._lon

    @lon.setter
    def lon(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lon' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'lon' field must be an integer in [-2147483648, 2147483647]"
        self._lon = value

    @builtins.property
    def alt(self):
        """Message field 'alt'."""
        return self._alt

    @alt.setter
    def alt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'alt' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'alt' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._alt = value

    @builtins.property
    def hdop(self):
        """Message field 'hdop'."""
        return self._hdop

    @hdop.setter
    def hdop(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'hdop' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'hdop' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._hdop = value

    @builtins.property
    def vdop(self):
        """Message field 'vdop'."""
        return self._vdop

    @vdop.setter
    def vdop(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vdop' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vdop' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vdop = value

    @builtins.property
    def vn(self):
        """Message field 'vn'."""
        return self._vn

    @vn.setter
    def vn(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vn' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vn' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vn = value

    @builtins.property
    def ve(self):
        """Message field 've'."""
        return self._ve

    @ve.setter
    def ve(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 've' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 've' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ve = value

    @builtins.property
    def vd(self):
        """Message field 'vd'."""
        return self._vd

    @vd.setter
    def vd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vd = value

    @builtins.property
    def speed_accuracy(self):
        """Message field 'speed_accuracy'."""
        return self._speed_accuracy

    @speed_accuracy.setter
    def speed_accuracy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_accuracy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_accuracy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_accuracy = value

    @builtins.property
    def horiz_accuracy(self):
        """Message field 'horiz_accuracy'."""
        return self._horiz_accuracy

    @horiz_accuracy.setter
    def horiz_accuracy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'horiz_accuracy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'horiz_accuracy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._horiz_accuracy = value

    @builtins.property
    def vert_accuracy(self):
        """Message field 'vert_accuracy'."""
        return self._vert_accuracy

    @vert_accuracy.setter
    def vert_accuracy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vert_accuracy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vert_accuracy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vert_accuracy = value

    @builtins.property
    def satellites_visible(self):
        """Message field 'satellites_visible'."""
        return self._satellites_visible

    @satellites_visible.setter
    def satellites_visible(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'satellites_visible' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'satellites_visible' field must be an unsigned integer in [0, 255]"
        self._satellites_visible = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'yaw' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'yaw' field must be an unsigned integer in [0, 65535]"
        self._yaw = value
