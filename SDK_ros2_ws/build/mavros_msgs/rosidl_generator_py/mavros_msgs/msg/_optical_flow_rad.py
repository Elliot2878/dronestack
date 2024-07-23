# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/OpticalFlowRad.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OpticalFlowRad(type):
    """Metaclass of message 'OpticalFlowRad'."""

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
                'mavros_msgs.msg.OpticalFlowRad')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__optical_flow_rad
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__optical_flow_rad
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__optical_flow_rad
            cls._TYPE_SUPPORT = module.type_support_msg__msg__optical_flow_rad
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__optical_flow_rad

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


class OpticalFlowRad(metaclass=Metaclass_OpticalFlowRad):
    """Message class 'OpticalFlowRad'."""

    __slots__ = [
        '_header',
        '_integration_time_us',
        '_integrated_x',
        '_integrated_y',
        '_integrated_xgyro',
        '_integrated_ygyro',
        '_integrated_zgyro',
        '_temperature',
        '_quality',
        '_time_delta_distance_us',
        '_distance',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'integration_time_us': 'uint32',
        'integrated_x': 'float',
        'integrated_y': 'float',
        'integrated_xgyro': 'float',
        'integrated_ygyro': 'float',
        'integrated_zgyro': 'float',
        'temperature': 'int16',
        'quality': 'uint8',
        'time_delta_distance_us': 'uint32',
        'distance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.integration_time_us = kwargs.get('integration_time_us', int())
        self.integrated_x = kwargs.get('integrated_x', float())
        self.integrated_y = kwargs.get('integrated_y', float())
        self.integrated_xgyro = kwargs.get('integrated_xgyro', float())
        self.integrated_ygyro = kwargs.get('integrated_ygyro', float())
        self.integrated_zgyro = kwargs.get('integrated_zgyro', float())
        self.temperature = kwargs.get('temperature', int())
        self.quality = kwargs.get('quality', int())
        self.time_delta_distance_us = kwargs.get('time_delta_distance_us', int())
        self.distance = kwargs.get('distance', float())

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
        if self.integration_time_us != other.integration_time_us:
            return False
        if self.integrated_x != other.integrated_x:
            return False
        if self.integrated_y != other.integrated_y:
            return False
        if self.integrated_xgyro != other.integrated_xgyro:
            return False
        if self.integrated_ygyro != other.integrated_ygyro:
            return False
        if self.integrated_zgyro != other.integrated_zgyro:
            return False
        if self.temperature != other.temperature:
            return False
        if self.quality != other.quality:
            return False
        if self.time_delta_distance_us != other.time_delta_distance_us:
            return False
        if self.distance != other.distance:
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
    def integration_time_us(self):
        """Message field 'integration_time_us'."""
        return self._integration_time_us

    @integration_time_us.setter
    def integration_time_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'integration_time_us' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'integration_time_us' field must be an unsigned integer in [0, 4294967295]"
        self._integration_time_us = value

    @builtins.property
    def integrated_x(self):
        """Message field 'integrated_x'."""
        return self._integrated_x

    @integrated_x.setter
    def integrated_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integrated_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'integrated_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._integrated_x = value

    @builtins.property
    def integrated_y(self):
        """Message field 'integrated_y'."""
        return self._integrated_y

    @integrated_y.setter
    def integrated_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integrated_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'integrated_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._integrated_y = value

    @builtins.property
    def integrated_xgyro(self):
        """Message field 'integrated_xgyro'."""
        return self._integrated_xgyro

    @integrated_xgyro.setter
    def integrated_xgyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integrated_xgyro' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'integrated_xgyro' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._integrated_xgyro = value

    @builtins.property
    def integrated_ygyro(self):
        """Message field 'integrated_ygyro'."""
        return self._integrated_ygyro

    @integrated_ygyro.setter
    def integrated_ygyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integrated_ygyro' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'integrated_ygyro' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._integrated_ygyro = value

    @builtins.property
    def integrated_zgyro(self):
        """Message field 'integrated_zgyro'."""
        return self._integrated_zgyro

    @integrated_zgyro.setter
    def integrated_zgyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integrated_zgyro' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'integrated_zgyro' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._integrated_zgyro = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'temperature' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'temperature' field must be an integer in [-32768, 32767]"
        self._temperature = value

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
    def time_delta_distance_us(self):
        """Message field 'time_delta_distance_us'."""
        return self._time_delta_distance_us

    @time_delta_distance_us.setter
    def time_delta_distance_us(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_delta_distance_us' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_delta_distance_us' field must be an unsigned integer in [0, 4294967295]"
        self._time_delta_distance_us = value

    @builtins.property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance = value
