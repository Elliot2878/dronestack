# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/HilGPS.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HilGPS(type):
    """Metaclass of message 'HilGPS'."""

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
                'mavros_msgs.msg.HilGPS')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hil_gps
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hil_gps
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hil_gps
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hil_gps
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hil_gps

            from geographic_msgs.msg import GeoPoint
            if GeoPoint.__class__._TYPE_SUPPORT is None:
                GeoPoint.__class__.__import_type_support__()

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


class HilGPS(metaclass=Metaclass_HilGPS):
    """Message class 'HilGPS'."""

    __slots__ = [
        '_header',
        '_fix_type',
        '_geo',
        '_eph',
        '_epv',
        '_vel',
        '_vn',
        '_ve',
        '_vd',
        '_cog',
        '_satellites_visible',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'fix_type': 'uint8',
        'geo': 'geographic_msgs/GeoPoint',
        'eph': 'uint16',
        'epv': 'uint16',
        'vel': 'uint16',
        'vn': 'int16',
        've': 'int16',
        'vd': 'int16',
        'cog': 'uint16',
        'satellites_visible': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geographic_msgs', 'msg'], 'GeoPoint'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.fix_type = kwargs.get('fix_type', int())
        from geographic_msgs.msg import GeoPoint
        self.geo = kwargs.get('geo', GeoPoint())
        self.eph = kwargs.get('eph', int())
        self.epv = kwargs.get('epv', int())
        self.vel = kwargs.get('vel', int())
        self.vn = kwargs.get('vn', int())
        self.ve = kwargs.get('ve', int())
        self.vd = kwargs.get('vd', int())
        self.cog = kwargs.get('cog', int())
        self.satellites_visible = kwargs.get('satellites_visible', int())

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
        if self.geo != other.geo:
            return False
        if self.eph != other.eph:
            return False
        if self.epv != other.epv:
            return False
        if self.vel != other.vel:
            return False
        if self.vn != other.vn:
            return False
        if self.ve != other.ve:
            return False
        if self.vd != other.vd:
            return False
        if self.cog != other.cog:
            return False
        if self.satellites_visible != other.satellites_visible:
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
    def vn(self):
        """Message field 'vn'."""
        return self._vn

    @vn.setter
    def vn(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vn' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'vn' field must be an integer in [-32768, 32767]"
        self._vn = value

    @builtins.property
    def ve(self):
        """Message field 've'."""
        return self._ve

    @ve.setter
    def ve(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 've' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 've' field must be an integer in [-32768, 32767]"
        self._ve = value

    @builtins.property
    def vd(self):
        """Message field 'vd'."""
        return self._vd

    @vd.setter
    def vd(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vd' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'vd' field must be an integer in [-32768, 32767]"
        self._vd = value

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
