# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/HilSensor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HilSensor(type):
    """Metaclass of message 'HilSensor'."""

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
                'mavros_msgs.msg.HilSensor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hil_sensor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hil_sensor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hil_sensor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hil_sensor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hil_sensor

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

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


class HilSensor(metaclass=Metaclass_HilSensor):
    """Message class 'HilSensor'."""

    __slots__ = [
        '_header',
        '_acc',
        '_gyro',
        '_mag',
        '_abs_pressure',
        '_diff_pressure',
        '_pressure_alt',
        '_temperature',
        '_fields_updated',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'acc': 'geometry_msgs/Vector3',
        'gyro': 'geometry_msgs/Vector3',
        'mag': 'geometry_msgs/Vector3',
        'abs_pressure': 'float',
        'diff_pressure': 'float',
        'pressure_alt': 'float',
        'temperature': 'float',
        'fields_updated': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Vector3
        self.acc = kwargs.get('acc', Vector3())
        from geometry_msgs.msg import Vector3
        self.gyro = kwargs.get('gyro', Vector3())
        from geometry_msgs.msg import Vector3
        self.mag = kwargs.get('mag', Vector3())
        self.abs_pressure = kwargs.get('abs_pressure', float())
        self.diff_pressure = kwargs.get('diff_pressure', float())
        self.pressure_alt = kwargs.get('pressure_alt', float())
        self.temperature = kwargs.get('temperature', float())
        self.fields_updated = kwargs.get('fields_updated', int())

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
        if self.acc != other.acc:
            return False
        if self.gyro != other.gyro:
            return False
        if self.mag != other.mag:
            return False
        if self.abs_pressure != other.abs_pressure:
            return False
        if self.diff_pressure != other.diff_pressure:
            return False
        if self.pressure_alt != other.pressure_alt:
            return False
        if self.temperature != other.temperature:
            return False
        if self.fields_updated != other.fields_updated:
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
    def acc(self):
        """Message field 'acc'."""
        return self._acc

    @acc.setter
    def acc(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'acc' field must be a sub message of type 'Vector3'"
        self._acc = value

    @builtins.property
    def gyro(self):
        """Message field 'gyro'."""
        return self._gyro

    @gyro.setter
    def gyro(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'gyro' field must be a sub message of type 'Vector3'"
        self._gyro = value

    @builtins.property
    def mag(self):
        """Message field 'mag'."""
        return self._mag

    @mag.setter
    def mag(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'mag' field must be a sub message of type 'Vector3'"
        self._mag = value

    @builtins.property
    def abs_pressure(self):
        """Message field 'abs_pressure'."""
        return self._abs_pressure

    @abs_pressure.setter
    def abs_pressure(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'abs_pressure' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'abs_pressure' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._abs_pressure = value

    @builtins.property
    def diff_pressure(self):
        """Message field 'diff_pressure'."""
        return self._diff_pressure

    @diff_pressure.setter
    def diff_pressure(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'diff_pressure' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'diff_pressure' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._diff_pressure = value

    @builtins.property
    def pressure_alt(self):
        """Message field 'pressure_alt'."""
        return self._pressure_alt

    @pressure_alt.setter
    def pressure_alt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pressure_alt' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pressure_alt' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pressure_alt = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._temperature = value

    @builtins.property
    def fields_updated(self):
        """Message field 'fields_updated'."""
        return self._fields_updated

    @fields_updated.setter
    def fields_updated(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fields_updated' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'fields_updated' field must be an unsigned integer in [0, 4294967295]"
        self._fields_updated = value
