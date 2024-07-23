# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/GimbalDeviceInformation.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalDeviceInformation(type):
    """Metaclass of message 'GimbalDeviceInformation'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CAP_FLAGS_HAS_RETRACT': 1,
        'CAP_FLAGS_HAS_NEUTRAL': 2,
        'CAP_FLAGS_HAS_ROLL_AXIS': 4,
        'CAP_FLAGS_HAS_ROLL_FOLLOW': 8,
        'CAP_FLAGS_HAS_ROLL_LOCK': 16,
        'CAP_FLAGS_HAS_PITCH_AXIS': 32,
        'CAP_FLAGS_HAS_PITCH_FOLLOW': 64,
        'CAP_FLAGS_HAS_PITCH_LOCK': 128,
        'CAP_FLAGS_HAS_YAW_AXIS': 256,
        'CAP_FLAGS_HAS_YAW_FOLLOW': 512,
        'CAP_FLAGS_HAS_YAW_LOCK': 1024,
        'CAP_FLAGS_SUPPORTS_INFINITE_YAW': 2048,
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
                'mavros_msgs.msg.GimbalDeviceInformation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gimbal_device_information
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gimbal_device_information
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gimbal_device_information
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gimbal_device_information
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gimbal_device_information

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CAP_FLAGS_HAS_RETRACT': cls.__constants['CAP_FLAGS_HAS_RETRACT'],
            'CAP_FLAGS_HAS_NEUTRAL': cls.__constants['CAP_FLAGS_HAS_NEUTRAL'],
            'CAP_FLAGS_HAS_ROLL_AXIS': cls.__constants['CAP_FLAGS_HAS_ROLL_AXIS'],
            'CAP_FLAGS_HAS_ROLL_FOLLOW': cls.__constants['CAP_FLAGS_HAS_ROLL_FOLLOW'],
            'CAP_FLAGS_HAS_ROLL_LOCK': cls.__constants['CAP_FLAGS_HAS_ROLL_LOCK'],
            'CAP_FLAGS_HAS_PITCH_AXIS': cls.__constants['CAP_FLAGS_HAS_PITCH_AXIS'],
            'CAP_FLAGS_HAS_PITCH_FOLLOW': cls.__constants['CAP_FLAGS_HAS_PITCH_FOLLOW'],
            'CAP_FLAGS_HAS_PITCH_LOCK': cls.__constants['CAP_FLAGS_HAS_PITCH_LOCK'],
            'CAP_FLAGS_HAS_YAW_AXIS': cls.__constants['CAP_FLAGS_HAS_YAW_AXIS'],
            'CAP_FLAGS_HAS_YAW_FOLLOW': cls.__constants['CAP_FLAGS_HAS_YAW_FOLLOW'],
            'CAP_FLAGS_HAS_YAW_LOCK': cls.__constants['CAP_FLAGS_HAS_YAW_LOCK'],
            'CAP_FLAGS_SUPPORTS_INFINITE_YAW': cls.__constants['CAP_FLAGS_SUPPORTS_INFINITE_YAW'],
        }

    @property
    def CAP_FLAGS_HAS_RETRACT(self):
        """Message constant 'CAP_FLAGS_HAS_RETRACT'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_RETRACT']

    @property
    def CAP_FLAGS_HAS_NEUTRAL(self):
        """Message constant 'CAP_FLAGS_HAS_NEUTRAL'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_NEUTRAL']

    @property
    def CAP_FLAGS_HAS_ROLL_AXIS(self):
        """Message constant 'CAP_FLAGS_HAS_ROLL_AXIS'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_ROLL_AXIS']

    @property
    def CAP_FLAGS_HAS_ROLL_FOLLOW(self):
        """Message constant 'CAP_FLAGS_HAS_ROLL_FOLLOW'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_ROLL_FOLLOW']

    @property
    def CAP_FLAGS_HAS_ROLL_LOCK(self):
        """Message constant 'CAP_FLAGS_HAS_ROLL_LOCK'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_ROLL_LOCK']

    @property
    def CAP_FLAGS_HAS_PITCH_AXIS(self):
        """Message constant 'CAP_FLAGS_HAS_PITCH_AXIS'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_PITCH_AXIS']

    @property
    def CAP_FLAGS_HAS_PITCH_FOLLOW(self):
        """Message constant 'CAP_FLAGS_HAS_PITCH_FOLLOW'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_PITCH_FOLLOW']

    @property
    def CAP_FLAGS_HAS_PITCH_LOCK(self):
        """Message constant 'CAP_FLAGS_HAS_PITCH_LOCK'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_PITCH_LOCK']

    @property
    def CAP_FLAGS_HAS_YAW_AXIS(self):
        """Message constant 'CAP_FLAGS_HAS_YAW_AXIS'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_YAW_AXIS']

    @property
    def CAP_FLAGS_HAS_YAW_FOLLOW(self):
        """Message constant 'CAP_FLAGS_HAS_YAW_FOLLOW'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_YAW_FOLLOW']

    @property
    def CAP_FLAGS_HAS_YAW_LOCK(self):
        """Message constant 'CAP_FLAGS_HAS_YAW_LOCK'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_HAS_YAW_LOCK']

    @property
    def CAP_FLAGS_SUPPORTS_INFINITE_YAW(self):
        """Message constant 'CAP_FLAGS_SUPPORTS_INFINITE_YAW'."""
        return Metaclass_GimbalDeviceInformation.__constants['CAP_FLAGS_SUPPORTS_INFINITE_YAW']


class GimbalDeviceInformation(metaclass=Metaclass_GimbalDeviceInformation):
    """
    Message class 'GimbalDeviceInformation'.

    Constants:
      CAP_FLAGS_HAS_RETRACT
      CAP_FLAGS_HAS_NEUTRAL
      CAP_FLAGS_HAS_ROLL_AXIS
      CAP_FLAGS_HAS_ROLL_FOLLOW
      CAP_FLAGS_HAS_ROLL_LOCK
      CAP_FLAGS_HAS_PITCH_AXIS
      CAP_FLAGS_HAS_PITCH_FOLLOW
      CAP_FLAGS_HAS_PITCH_LOCK
      CAP_FLAGS_HAS_YAW_AXIS
      CAP_FLAGS_HAS_YAW_FOLLOW
      CAP_FLAGS_HAS_YAW_LOCK
      CAP_FLAGS_SUPPORTS_INFINITE_YAW
    """

    __slots__ = [
        '_header',
        '_vendor_name',
        '_model_name',
        '_custom_name',
        '_firmware_version',
        '_hardware_version',
        '_uid',
        '_cap_flags',
        '_custom_cap_flags',
        '_roll_min',
        '_roll_max',
        '_pitch_min',
        '_pitch_max',
        '_yaw_min',
        '_yaw_max',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'vendor_name': 'string',
        'model_name': 'string',
        'custom_name': 'string',
        'firmware_version': 'uint32',
        'hardware_version': 'uint32',
        'uid': 'uint64',
        'cap_flags': 'uint32',
        'custom_cap_flags': 'uint16',
        'roll_min': 'float',
        'roll_max': 'float',
        'pitch_min': 'float',
        'pitch_max': 'float',
        'yaw_min': 'float',
        'yaw_max': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
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
        self.vendor_name = kwargs.get('vendor_name', str())
        self.model_name = kwargs.get('model_name', str())
        self.custom_name = kwargs.get('custom_name', str())
        self.firmware_version = kwargs.get('firmware_version', int())
        self.hardware_version = kwargs.get('hardware_version', int())
        self.uid = kwargs.get('uid', int())
        self.cap_flags = kwargs.get('cap_flags', int())
        self.custom_cap_flags = kwargs.get('custom_cap_flags', int())
        self.roll_min = kwargs.get('roll_min', float())
        self.roll_max = kwargs.get('roll_max', float())
        self.pitch_min = kwargs.get('pitch_min', float())
        self.pitch_max = kwargs.get('pitch_max', float())
        self.yaw_min = kwargs.get('yaw_min', float())
        self.yaw_max = kwargs.get('yaw_max', float())

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
        if self.vendor_name != other.vendor_name:
            return False
        if self.model_name != other.model_name:
            return False
        if self.custom_name != other.custom_name:
            return False
        if self.firmware_version != other.firmware_version:
            return False
        if self.hardware_version != other.hardware_version:
            return False
        if self.uid != other.uid:
            return False
        if self.cap_flags != other.cap_flags:
            return False
        if self.custom_cap_flags != other.custom_cap_flags:
            return False
        if self.roll_min != other.roll_min:
            return False
        if self.roll_max != other.roll_max:
            return False
        if self.pitch_min != other.pitch_min:
            return False
        if self.pitch_max != other.pitch_max:
            return False
        if self.yaw_min != other.yaw_min:
            return False
        if self.yaw_max != other.yaw_max:
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
    def vendor_name(self):
        """Message field 'vendor_name'."""
        return self._vendor_name

    @vendor_name.setter
    def vendor_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'vendor_name' field must be of type 'str'"
        self._vendor_name = value

    @builtins.property
    def model_name(self):
        """Message field 'model_name'."""
        return self._model_name

    @model_name.setter
    def model_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'model_name' field must be of type 'str'"
        self._model_name = value

    @builtins.property
    def custom_name(self):
        """Message field 'custom_name'."""
        return self._custom_name

    @custom_name.setter
    def custom_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'custom_name' field must be of type 'str'"
        self._custom_name = value

    @builtins.property
    def firmware_version(self):
        """Message field 'firmware_version'."""
        return self._firmware_version

    @firmware_version.setter
    def firmware_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'firmware_version' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'firmware_version' field must be an unsigned integer in [0, 4294967295]"
        self._firmware_version = value

    @builtins.property
    def hardware_version(self):
        """Message field 'hardware_version'."""
        return self._hardware_version

    @hardware_version.setter
    def hardware_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hardware_version' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'hardware_version' field must be an unsigned integer in [0, 4294967295]"
        self._hardware_version = value

    @builtins.property
    def uid(self):
        """Message field 'uid'."""
        return self._uid

    @uid.setter
    def uid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'uid' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'uid' field must be an unsigned integer in [0, 18446744073709551615]"
        self._uid = value

    @builtins.property
    def cap_flags(self):
        """Message field 'cap_flags'."""
        return self._cap_flags

    @cap_flags.setter
    def cap_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cap_flags' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'cap_flags' field must be an unsigned integer in [0, 4294967295]"
        self._cap_flags = value

    @builtins.property
    def custom_cap_flags(self):
        """Message field 'custom_cap_flags'."""
        return self._custom_cap_flags

    @custom_cap_flags.setter
    def custom_cap_flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'custom_cap_flags' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'custom_cap_flags' field must be an unsigned integer in [0, 65535]"
        self._custom_cap_flags = value

    @builtins.property
    def roll_min(self):
        """Message field 'roll_min'."""
        return self._roll_min

    @roll_min.setter
    def roll_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_min = value

    @builtins.property
    def roll_max(self):
        """Message field 'roll_max'."""
        return self._roll_max

    @roll_max.setter
    def roll_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_max = value

    @builtins.property
    def pitch_min(self):
        """Message field 'pitch_min'."""
        return self._pitch_min

    @pitch_min.setter
    def pitch_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_min = value

    @builtins.property
    def pitch_max(self):
        """Message field 'pitch_max'."""
        return self._pitch_max

    @pitch_max.setter
    def pitch_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_max = value

    @builtins.property
    def yaw_min(self):
        """Message field 'yaw_min'."""
        return self._yaw_min

    @yaw_min.setter
    def yaw_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_min = value

    @builtins.property
    def yaw_max(self):
        """Message field 'yaw_max'."""
        return self._yaw_max

    @yaw_max.setter
    def yaw_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_max = value
