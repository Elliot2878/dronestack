# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/GPSRAW.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GPSRAW(type):
    """Metaclass of message 'GPSRAW'."""

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
        'GPS_FIX_TYPE_RTK_FLOAT': 5,
        'GPS_FIX_TYPE_RTK_FIXED': 6,
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
                'mavros_msgs.msg.GPSRAW')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gpsraw
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gpsraw
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gpsraw
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gpsraw
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gpsraw

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
            'GPS_FIX_TYPE_RTK_FLOAT': cls.__constants['GPS_FIX_TYPE_RTK_FLOAT'],
            'GPS_FIX_TYPE_RTK_FIXED': cls.__constants['GPS_FIX_TYPE_RTK_FIXED'],
            'GPS_FIX_TYPE_STATIC': cls.__constants['GPS_FIX_TYPE_STATIC'],
            'GPS_FIX_TYPE_PPP': cls.__constants['GPS_FIX_TYPE_PPP'],
        }

    @property
    def GPS_FIX_TYPE_NO_GPS(self):
        """Message constant 'GPS_FIX_TYPE_NO_GPS'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_NO_GPS']

    @property
    def GPS_FIX_TYPE_NO_FIX(self):
        """Message constant 'GPS_FIX_TYPE_NO_FIX'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_NO_FIX']

    @property
    def GPS_FIX_TYPE_2D_FIX(self):
        """Message constant 'GPS_FIX_TYPE_2D_FIX'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_2D_FIX']

    @property
    def GPS_FIX_TYPE_3D_FIX(self):
        """Message constant 'GPS_FIX_TYPE_3D_FIX'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_3D_FIX']

    @property
    def GPS_FIX_TYPE_DGPS(self):
        """Message constant 'GPS_FIX_TYPE_DGPS'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_DGPS']

    @property
    def GPS_FIX_TYPE_RTK_FLOAT(self):
        """Message constant 'GPS_FIX_TYPE_RTK_FLOAT'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_RTK_FLOAT']

    @property
    def GPS_FIX_TYPE_RTK_FIXED(self):
        """Message constant 'GPS_FIX_TYPE_RTK_FIXED'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_RTK_FIXED']

    @property
    def GPS_FIX_TYPE_STATIC(self):
        """Message constant 'GPS_FIX_TYPE_STATIC'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_STATIC']

    @property
    def GPS_FIX_TYPE_PPP(self):
        """Message constant 'GPS_FIX_TYPE_PPP'."""
        return Metaclass_GPSRAW.__constants['GPS_FIX_TYPE_PPP']


class GPSRAW(metaclass=Metaclass_GPSRAW):
    """
    Message class 'GPSRAW'.

    Constants:
      GPS_FIX_TYPE_NO_GPS
      GPS_FIX_TYPE_NO_FIX
      GPS_FIX_TYPE_2D_FIX
      GPS_FIX_TYPE_3D_FIX
      GPS_FIX_TYPE_DGPS
      GPS_FIX_TYPE_RTK_FLOAT
      GPS_FIX_TYPE_RTK_FIXED
      GPS_FIX_TYPE_STATIC
      GPS_FIX_TYPE_PPP
    """

    __slots__ = [
        '_header',
        '_fix_type',
        '_lat',
        '_lon',
        '_alt',
        '_eph',
        '_epv',
        '_vel',
        '_cog',
        '_satellites_visible',
        '_alt_ellipsoid',
        '_h_acc',
        '_v_acc',
        '_vel_acc',
        '_hdg_acc',
        '_yaw',
        '_dgps_numch',
        '_dgps_age',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'fix_type': 'uint8',
        'lat': 'int32',
        'lon': 'int32',
        'alt': 'int32',
        'eph': 'uint16',
        'epv': 'uint16',
        'vel': 'uint16',
        'cog': 'uint16',
        'satellites_visible': 'uint8',
        'alt_ellipsoid': 'int32',
        'h_acc': 'uint32',
        'v_acc': 'uint32',
        'vel_acc': 'uint32',
        'hdg_acc': 'int32',
        'yaw': 'uint16',
        'dgps_numch': 'uint8',
        'dgps_age': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.fix_type = kwargs.get('fix_type', int())
        self.lat = kwargs.get('lat', int())
        self.lon = kwargs.get('lon', int())
        self.alt = kwargs.get('alt', int())
        self.eph = kwargs.get('eph', int())
        self.epv = kwargs.get('epv', int())
        self.vel = kwargs.get('vel', int())
        self.cog = kwargs.get('cog', int())
        self.satellites_visible = kwargs.get('satellites_visible', int())
        self.alt_ellipsoid = kwargs.get('alt_ellipsoid', int())
        self.h_acc = kwargs.get('h_acc', int())
        self.v_acc = kwargs.get('v_acc', int())
        self.vel_acc = kwargs.get('vel_acc', int())
        self.hdg_acc = kwargs.get('hdg_acc', int())
        self.yaw = kwargs.get('yaw', int())
        self.dgps_numch = kwargs.get('dgps_numch', int())
        self.dgps_age = kwargs.get('dgps_age', int())

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
        if self.lat != other.lat:
            return False
        if self.lon != other.lon:
            return False
        if self.alt != other.alt:
            return False
        if self.eph != other.eph:
            return False
        if self.epv != other.epv:
            return False
        if self.vel != other.vel:
            return False
        if self.cog != other.cog:
            return False
        if self.satellites_visible != other.satellites_visible:
            return False
        if self.alt_ellipsoid != other.alt_ellipsoid:
            return False
        if self.h_acc != other.h_acc:
            return False
        if self.v_acc != other.v_acc:
            return False
        if self.vel_acc != other.vel_acc:
            return False
        if self.hdg_acc != other.hdg_acc:
            return False
        if self.yaw != other.yaw:
            return False
        if self.dgps_numch != other.dgps_numch:
            return False
        if self.dgps_age != other.dgps_age:
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
                isinstance(value, int), \
                "The 'alt' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'alt' field must be an integer in [-2147483648, 2147483647]"
        self._alt = value

    @builtins.property
    def eph(self):
        """Message field 'eph'."""
        return self._eph

    @eph.setter
    def eph(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'eph' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'eph' field must be an unsigned integer in [0, 65535]"
        self._eph = value

    @builtins.property
    def epv(self):
        """Message field 'epv'."""
        return self._epv

    @epv.setter
    def epv(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'epv' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'epv' field must be an unsigned integer in [0, 65535]"
        self._epv = value

    @builtins.property
    def vel(self):
        """Message field 'vel'."""
        return self._vel

    @vel.setter
    def vel(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vel' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'vel' field must be an unsigned integer in [0, 65535]"
        self._vel = value

    @builtins.property
    def cog(self):
        """Message field 'cog'."""
        return self._cog

    @cog.setter
    def cog(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cog' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'cog' field must be an unsigned integer in [0, 65535]"
        self._cog = value

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
    def alt_ellipsoid(self):
        """Message field 'alt_ellipsoid'."""
        return self._alt_ellipsoid

    @alt_ellipsoid.setter
    def alt_ellipsoid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'alt_ellipsoid' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'alt_ellipsoid' field must be an integer in [-2147483648, 2147483647]"
        self._alt_ellipsoid = value

    @builtins.property
    def h_acc(self):
        """Message field 'h_acc'."""
        return self._h_acc

    @h_acc.setter
    def h_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'h_acc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'h_acc' field must be an unsigned integer in [0, 4294967295]"
        self._h_acc = value

    @builtins.property
    def v_acc(self):
        """Message field 'v_acc'."""
        return self._v_acc

    @v_acc.setter
    def v_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'v_acc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'v_acc' field must be an unsigned integer in [0, 4294967295]"
        self._v_acc = value

    @builtins.property
    def vel_acc(self):
        """Message field 'vel_acc'."""
        return self._vel_acc

    @vel_acc.setter
    def vel_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vel_acc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'vel_acc' field must be an unsigned integer in [0, 4294967295]"
        self._vel_acc = value

    @builtins.property
    def hdg_acc(self):
        """Message field 'hdg_acc'."""
        return self._hdg_acc

    @hdg_acc.setter
    def hdg_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hdg_acc' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'hdg_acc' field must be an integer in [-2147483648, 2147483647]"
        self._hdg_acc = value

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

    @builtins.property
    def dgps_numch(self):
        """Message field 'dgps_numch'."""
        return self._dgps_numch

    @dgps_numch.setter
    def dgps_numch(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'dgps_numch' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'dgps_numch' field must be an unsigned integer in [0, 255]"
        self._dgps_numch = value

    @builtins.property
    def dgps_age(self):
        """Message field 'dgps_age'."""
        return self._dgps_age

    @dgps_age.setter
    def dgps_age(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'dgps_age' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'dgps_age' field must be an unsigned integer in [0, 4294967295]"
        self._dgps_age = value
