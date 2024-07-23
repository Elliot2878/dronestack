# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/ADSBVehicle.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ADSBVehicle(type):
    """Metaclass of message 'ADSBVehicle'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ALT_PRESSURE_QNH': 0,
        'ALT_GEOMETRIC': 1,
        'EMITTER_NO_INFO': 0,
        'EMITTER_LIGHT': 1,
        'EMITTER_SMALL': 2,
        'EMITTER_LARGE': 3,
        'EMITTER_HIGH_VORTEX_LARGE': 4,
        'EMITTER_HEAVY': 5,
        'EMITTER_HIGHLY_MANUV': 6,
        'EMITTER_ROTOCRAFT': 7,
        'EMITTER_UNASSIGNED': 8,
        'EMITTER_GLIDER': 9,
        'EMITTER_LIGHTER_AIR': 10,
        'EMITTER_PARACHUTE': 11,
        'EMITTER_ULTRA_LIGHT': 12,
        'EMITTER_UNASSIGNED2': 13,
        'EMITTER_UAV': 14,
        'EMITTER_SPACE': 15,
        'EMITTER_UNASSGINED3': 16,
        'EMITTER_EMERGENCY_SURFACE': 17,
        'EMITTER_SERVICE_SURFACE': 18,
        'EMITTER_POINT_OBSTACLE': 19,
        'FLAG_VALID_COORDS': 1,
        'FLAG_VALID_ALTITUDE': 2,
        'FLAG_VALID_HEADING': 4,
        'FLAG_VALID_VELOCITY': 8,
        'FLAG_VALID_CALLSIGN': 16,
        'FLAG_VALID_SQUAWK': 32,
        'FLAG_SIMULATED': 64,
        'FLAG_VERTICAL_VELOCITY_VALID': 128,
        'FLAG_BARO_VALID': 256,
        'FLAG_SOURCE_UAT': 32768,
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
                'mavros_msgs.msg.ADSBVehicle')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__adsb_vehicle
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__adsb_vehicle
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__adsb_vehicle
            cls._TYPE_SUPPORT = module.type_support_msg__msg__adsb_vehicle
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__adsb_vehicle

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ALT_PRESSURE_QNH': cls.__constants['ALT_PRESSURE_QNH'],
            'ALT_GEOMETRIC': cls.__constants['ALT_GEOMETRIC'],
            'EMITTER_NO_INFO': cls.__constants['EMITTER_NO_INFO'],
            'EMITTER_LIGHT': cls.__constants['EMITTER_LIGHT'],
            'EMITTER_SMALL': cls.__constants['EMITTER_SMALL'],
            'EMITTER_LARGE': cls.__constants['EMITTER_LARGE'],
            'EMITTER_HIGH_VORTEX_LARGE': cls.__constants['EMITTER_HIGH_VORTEX_LARGE'],
            'EMITTER_HEAVY': cls.__constants['EMITTER_HEAVY'],
            'EMITTER_HIGHLY_MANUV': cls.__constants['EMITTER_HIGHLY_MANUV'],
            'EMITTER_ROTOCRAFT': cls.__constants['EMITTER_ROTOCRAFT'],
            'EMITTER_UNASSIGNED': cls.__constants['EMITTER_UNASSIGNED'],
            'EMITTER_GLIDER': cls.__constants['EMITTER_GLIDER'],
            'EMITTER_LIGHTER_AIR': cls.__constants['EMITTER_LIGHTER_AIR'],
            'EMITTER_PARACHUTE': cls.__constants['EMITTER_PARACHUTE'],
            'EMITTER_ULTRA_LIGHT': cls.__constants['EMITTER_ULTRA_LIGHT'],
            'EMITTER_UNASSIGNED2': cls.__constants['EMITTER_UNASSIGNED2'],
            'EMITTER_UAV': cls.__constants['EMITTER_UAV'],
            'EMITTER_SPACE': cls.__constants['EMITTER_SPACE'],
            'EMITTER_UNASSGINED3': cls.__constants['EMITTER_UNASSGINED3'],
            'EMITTER_EMERGENCY_SURFACE': cls.__constants['EMITTER_EMERGENCY_SURFACE'],
            'EMITTER_SERVICE_SURFACE': cls.__constants['EMITTER_SERVICE_SURFACE'],
            'EMITTER_POINT_OBSTACLE': cls.__constants['EMITTER_POINT_OBSTACLE'],
            'FLAG_VALID_COORDS': cls.__constants['FLAG_VALID_COORDS'],
            'FLAG_VALID_ALTITUDE': cls.__constants['FLAG_VALID_ALTITUDE'],
            'FLAG_VALID_HEADING': cls.__constants['FLAG_VALID_HEADING'],
            'FLAG_VALID_VELOCITY': cls.__constants['FLAG_VALID_VELOCITY'],
            'FLAG_VALID_CALLSIGN': cls.__constants['FLAG_VALID_CALLSIGN'],
            'FLAG_VALID_SQUAWK': cls.__constants['FLAG_VALID_SQUAWK'],
            'FLAG_SIMULATED': cls.__constants['FLAG_SIMULATED'],
            'FLAG_VERTICAL_VELOCITY_VALID': cls.__constants['FLAG_VERTICAL_VELOCITY_VALID'],
            'FLAG_BARO_VALID': cls.__constants['FLAG_BARO_VALID'],
            'FLAG_SOURCE_UAT': cls.__constants['FLAG_SOURCE_UAT'],
        }

    @property
    def ALT_PRESSURE_QNH(self):
        """Message constant 'ALT_PRESSURE_QNH'."""
        return Metaclass_ADSBVehicle.__constants['ALT_PRESSURE_QNH']

    @property
    def ALT_GEOMETRIC(self):
        """Message constant 'ALT_GEOMETRIC'."""
        return Metaclass_ADSBVehicle.__constants['ALT_GEOMETRIC']

    @property
    def EMITTER_NO_INFO(self):
        """Message constant 'EMITTER_NO_INFO'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_NO_INFO']

    @property
    def EMITTER_LIGHT(self):
        """Message constant 'EMITTER_LIGHT'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_LIGHT']

    @property
    def EMITTER_SMALL(self):
        """Message constant 'EMITTER_SMALL'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_SMALL']

    @property
    def EMITTER_LARGE(self):
        """Message constant 'EMITTER_LARGE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_LARGE']

    @property
    def EMITTER_HIGH_VORTEX_LARGE(self):
        """Message constant 'EMITTER_HIGH_VORTEX_LARGE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_HIGH_VORTEX_LARGE']

    @property
    def EMITTER_HEAVY(self):
        """Message constant 'EMITTER_HEAVY'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_HEAVY']

    @property
    def EMITTER_HIGHLY_MANUV(self):
        """Message constant 'EMITTER_HIGHLY_MANUV'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_HIGHLY_MANUV']

    @property
    def EMITTER_ROTOCRAFT(self):
        """Message constant 'EMITTER_ROTOCRAFT'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_ROTOCRAFT']

    @property
    def EMITTER_UNASSIGNED(self):
        """Message constant 'EMITTER_UNASSIGNED'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_UNASSIGNED']

    @property
    def EMITTER_GLIDER(self):
        """Message constant 'EMITTER_GLIDER'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_GLIDER']

    @property
    def EMITTER_LIGHTER_AIR(self):
        """Message constant 'EMITTER_LIGHTER_AIR'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_LIGHTER_AIR']

    @property
    def EMITTER_PARACHUTE(self):
        """Message constant 'EMITTER_PARACHUTE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_PARACHUTE']

    @property
    def EMITTER_ULTRA_LIGHT(self):
        """Message constant 'EMITTER_ULTRA_LIGHT'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_ULTRA_LIGHT']

    @property
    def EMITTER_UNASSIGNED2(self):
        """Message constant 'EMITTER_UNASSIGNED2'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_UNASSIGNED2']

    @property
    def EMITTER_UAV(self):
        """Message constant 'EMITTER_UAV'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_UAV']

    @property
    def EMITTER_SPACE(self):
        """Message constant 'EMITTER_SPACE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_SPACE']

    @property
    def EMITTER_UNASSGINED3(self):
        """Message constant 'EMITTER_UNASSGINED3'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_UNASSGINED3']

    @property
    def EMITTER_EMERGENCY_SURFACE(self):
        """Message constant 'EMITTER_EMERGENCY_SURFACE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_EMERGENCY_SURFACE']

    @property
    def EMITTER_SERVICE_SURFACE(self):
        """Message constant 'EMITTER_SERVICE_SURFACE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_SERVICE_SURFACE']

    @property
    def EMITTER_POINT_OBSTACLE(self):
        """Message constant 'EMITTER_POINT_OBSTACLE'."""
        return Metaclass_ADSBVehicle.__constants['EMITTER_POINT_OBSTACLE']

    @property
    def FLAG_VALID_COORDS(self):
        """Message constant 'FLAG_VALID_COORDS'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VALID_COORDS']

    @property
    def FLAG_VALID_ALTITUDE(self):
        """Message constant 'FLAG_VALID_ALTITUDE'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VALID_ALTITUDE']

    @property
    def FLAG_VALID_HEADING(self):
        """Message constant 'FLAG_VALID_HEADING'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VALID_HEADING']

    @property
    def FLAG_VALID_VELOCITY(self):
        """Message constant 'FLAG_VALID_VELOCITY'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VALID_VELOCITY']

    @property
    def FLAG_VALID_CALLSIGN(self):
        """Message constant 'FLAG_VALID_CALLSIGN'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VALID_CALLSIGN']

    @property
    def FLAG_VALID_SQUAWK(self):
        """Message constant 'FLAG_VALID_SQUAWK'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VALID_SQUAWK']

    @property
    def FLAG_SIMULATED(self):
        """Message constant 'FLAG_SIMULATED'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_SIMULATED']

    @property
    def FLAG_VERTICAL_VELOCITY_VALID(self):
        """Message constant 'FLAG_VERTICAL_VELOCITY_VALID'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_VERTICAL_VELOCITY_VALID']

    @property
    def FLAG_BARO_VALID(self):
        """Message constant 'FLAG_BARO_VALID'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_BARO_VALID']

    @property
    def FLAG_SOURCE_UAT(self):
        """Message constant 'FLAG_SOURCE_UAT'."""
        return Metaclass_ADSBVehicle.__constants['FLAG_SOURCE_UAT']


class ADSBVehicle(metaclass=Metaclass_ADSBVehicle):
    """
    Message class 'ADSBVehicle'.

    Constants:
      ALT_PRESSURE_QNH
      ALT_GEOMETRIC
      EMITTER_NO_INFO
      EMITTER_LIGHT
      EMITTER_SMALL
      EMITTER_LARGE
      EMITTER_HIGH_VORTEX_LARGE
      EMITTER_HEAVY
      EMITTER_HIGHLY_MANUV
      EMITTER_ROTOCRAFT
      EMITTER_UNASSIGNED
      EMITTER_GLIDER
      EMITTER_LIGHTER_AIR
      EMITTER_PARACHUTE
      EMITTER_ULTRA_LIGHT
      EMITTER_UNASSIGNED2
      EMITTER_UAV
      EMITTER_SPACE
      EMITTER_UNASSGINED3
      EMITTER_EMERGENCY_SURFACE
      EMITTER_SERVICE_SURFACE
      EMITTER_POINT_OBSTACLE
      FLAG_VALID_COORDS
      FLAG_VALID_ALTITUDE
      FLAG_VALID_HEADING
      FLAG_VALID_VELOCITY
      FLAG_VALID_CALLSIGN
      FLAG_VALID_SQUAWK
      FLAG_SIMULATED
      FLAG_VERTICAL_VELOCITY_VALID
      FLAG_BARO_VALID
      FLAG_SOURCE_UAT
    """

    __slots__ = [
        '_header',
        '_icao_address',
        '_callsign',
        '_latitude',
        '_longitude',
        '_altitude',
        '_heading',
        '_hor_velocity',
        '_ver_velocity',
        '_altitude_type',
        '_emitter_type',
        '_tslc',
        '_flags',
        '_squawk',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'icao_address': 'uint32',
        'callsign': 'string',
        'latitude': 'double',
        'longitude': 'double',
        'altitude': 'float',
        'heading': 'float',
        'hor_velocity': 'float',
        'ver_velocity': 'float',
        'altitude_type': 'uint8',
        'emitter_type': 'uint8',
        'tslc': 'builtin_interfaces/Duration',
        'flags': 'uint16',
        'squawk': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.icao_address = kwargs.get('icao_address', int())
        self.callsign = kwargs.get('callsign', str())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.altitude = kwargs.get('altitude', float())
        self.heading = kwargs.get('heading', float())
        self.hor_velocity = kwargs.get('hor_velocity', float())
        self.ver_velocity = kwargs.get('ver_velocity', float())
        self.altitude_type = kwargs.get('altitude_type', int())
        self.emitter_type = kwargs.get('emitter_type', int())
        from builtin_interfaces.msg import Duration
        self.tslc = kwargs.get('tslc', Duration())
        self.flags = kwargs.get('flags', int())
        self.squawk = kwargs.get('squawk', int())

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
        if self.icao_address != other.icao_address:
            return False
        if self.callsign != other.callsign:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.heading != other.heading:
            return False
        if self.hor_velocity != other.hor_velocity:
            return False
        if self.ver_velocity != other.ver_velocity:
            return False
        if self.altitude_type != other.altitude_type:
            return False
        if self.emitter_type != other.emitter_type:
            return False
        if self.tslc != other.tslc:
            return False
        if self.flags != other.flags:
            return False
        if self.squawk != other.squawk:
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
    def icao_address(self):
        """Message field 'icao_address'."""
        return self._icao_address

    @icao_address.setter
    def icao_address(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'icao_address' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'icao_address' field must be an unsigned integer in [0, 4294967295]"
        self._icao_address = value

    @builtins.property
    def callsign(self):
        """Message field 'callsign'."""
        return self._callsign

    @callsign.setter
    def callsign(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'callsign' field must be of type 'str'"
        self._callsign = value

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
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'altitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._altitude = value

    @builtins.property
    def heading(self):
        """Message field 'heading'."""
        return self._heading

    @heading.setter
    def heading(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heading' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heading = value

    @builtins.property
    def hor_velocity(self):
        """Message field 'hor_velocity'."""
        return self._hor_velocity

    @hor_velocity.setter
    def hor_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'hor_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'hor_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._hor_velocity = value

    @builtins.property
    def ver_velocity(self):
        """Message field 'ver_velocity'."""
        return self._ver_velocity

    @ver_velocity.setter
    def ver_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ver_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ver_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ver_velocity = value

    @builtins.property
    def altitude_type(self):
        """Message field 'altitude_type'."""
        return self._altitude_type

    @altitude_type.setter
    def altitude_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'altitude_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'altitude_type' field must be an unsigned integer in [0, 255]"
        self._altitude_type = value

    @builtins.property
    def emitter_type(self):
        """Message field 'emitter_type'."""
        return self._emitter_type

    @emitter_type.setter
    def emitter_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'emitter_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'emitter_type' field must be an unsigned integer in [0, 255]"
        self._emitter_type = value

    @builtins.property
    def tslc(self):
        """Message field 'tslc'."""
        return self._tslc

    @tslc.setter
    def tslc(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'tslc' field must be a sub message of type 'Duration'"
        self._tslc = value

    @builtins.property
    def flags(self):
        """Message field 'flags'."""
        return self._flags

    @flags.setter
    def flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'flags' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'flags' field must be an unsigned integer in [0, 65535]"
        self._flags = value

    @builtins.property
    def squawk(self):
        """Message field 'squawk'."""
        return self._squawk

    @squawk.setter
    def squawk(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'squawk' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'squawk' field must be an unsigned integer in [0, 65535]"
        self._squawk = value
