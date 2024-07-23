# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/GimbalManagerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalManagerStatus(type):
    """Metaclass of message 'GimbalManagerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GIMBAL_MANAGER_FLAGS_RETRACT': 1,
        'GIMBAL_MANAGER_FLAGS_NEUTRAL': 2,
        'GIMBAL_MANAGER_FLAGS_ROLL_LOCK': 4,
        'GIMBAL_MANAGER_FLAGS_PITCH_LOCK': 8,
        'GIMBAL_MANAGER_FLAGS_YAW_LOCK': 16,
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
                'mavros_msgs.msg.GimbalManagerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gimbal_manager_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gimbal_manager_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gimbal_manager_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gimbal_manager_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gimbal_manager_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GIMBAL_MANAGER_FLAGS_RETRACT': cls.__constants['GIMBAL_MANAGER_FLAGS_RETRACT'],
            'GIMBAL_MANAGER_FLAGS_NEUTRAL': cls.__constants['GIMBAL_MANAGER_FLAGS_NEUTRAL'],
            'GIMBAL_MANAGER_FLAGS_ROLL_LOCK': cls.__constants['GIMBAL_MANAGER_FLAGS_ROLL_LOCK'],
            'GIMBAL_MANAGER_FLAGS_PITCH_LOCK': cls.__constants['GIMBAL_MANAGER_FLAGS_PITCH_LOCK'],
            'GIMBAL_MANAGER_FLAGS_YAW_LOCK': cls.__constants['GIMBAL_MANAGER_FLAGS_YAW_LOCK'],
        }

    @property
    def GIMBAL_MANAGER_FLAGS_RETRACT(self):
        """Message constant 'GIMBAL_MANAGER_FLAGS_RETRACT'."""
        return Metaclass_GimbalManagerStatus.__constants['GIMBAL_MANAGER_FLAGS_RETRACT']

    @property
    def GIMBAL_MANAGER_FLAGS_NEUTRAL(self):
        """Message constant 'GIMBAL_MANAGER_FLAGS_NEUTRAL'."""
        return Metaclass_GimbalManagerStatus.__constants['GIMBAL_MANAGER_FLAGS_NEUTRAL']

    @property
    def GIMBAL_MANAGER_FLAGS_ROLL_LOCK(self):
        """Message constant 'GIMBAL_MANAGER_FLAGS_ROLL_LOCK'."""
        return Metaclass_GimbalManagerStatus.__constants['GIMBAL_MANAGER_FLAGS_ROLL_LOCK']

    @property
    def GIMBAL_MANAGER_FLAGS_PITCH_LOCK(self):
        """Message constant 'GIMBAL_MANAGER_FLAGS_PITCH_LOCK'."""
        return Metaclass_GimbalManagerStatus.__constants['GIMBAL_MANAGER_FLAGS_PITCH_LOCK']

    @property
    def GIMBAL_MANAGER_FLAGS_YAW_LOCK(self):
        """Message constant 'GIMBAL_MANAGER_FLAGS_YAW_LOCK'."""
        return Metaclass_GimbalManagerStatus.__constants['GIMBAL_MANAGER_FLAGS_YAW_LOCK']


class GimbalManagerStatus(metaclass=Metaclass_GimbalManagerStatus):
    """
    Message class 'GimbalManagerStatus'.

    Constants:
      GIMBAL_MANAGER_FLAGS_RETRACT
      GIMBAL_MANAGER_FLAGS_NEUTRAL
      GIMBAL_MANAGER_FLAGS_ROLL_LOCK
      GIMBAL_MANAGER_FLAGS_PITCH_LOCK
      GIMBAL_MANAGER_FLAGS_YAW_LOCK
    """

    __slots__ = [
        '_header',
        '_flags',
        '_gimbal_device_id',
        '_sysid_primary',
        '_compid_primary',
        '_sysid_secondary',
        '_compid_secondary',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'flags': 'uint32',
        'gimbal_device_id': 'uint8',
        'sysid_primary': 'uint8',
        'compid_primary': 'uint8',
        'sysid_secondary': 'uint8',
        'compid_secondary': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.flags = kwargs.get('flags', int())
        self.gimbal_device_id = kwargs.get('gimbal_device_id', int())
        self.sysid_primary = kwargs.get('sysid_primary', int())
        self.compid_primary = kwargs.get('compid_primary', int())
        self.sysid_secondary = kwargs.get('sysid_secondary', int())
        self.compid_secondary = kwargs.get('compid_secondary', int())

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
        if self.flags != other.flags:
            return False
        if self.gimbal_device_id != other.gimbal_device_id:
            return False
        if self.sysid_primary != other.sysid_primary:
            return False
        if self.compid_primary != other.compid_primary:
            return False
        if self.sysid_secondary != other.sysid_secondary:
            return False
        if self.compid_secondary != other.compid_secondary:
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
    def flags(self):
        """Message field 'flags'."""
        return self._flags

    @flags.setter
    def flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'flags' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'flags' field must be an unsigned integer in [0, 4294967295]"
        self._flags = value

    @builtins.property
    def gimbal_device_id(self):
        """Message field 'gimbal_device_id'."""
        return self._gimbal_device_id

    @gimbal_device_id.setter
    def gimbal_device_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gimbal_device_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gimbal_device_id' field must be an unsigned integer in [0, 255]"
        self._gimbal_device_id = value

    @builtins.property
    def sysid_primary(self):
        """Message field 'sysid_primary'."""
        return self._sysid_primary

    @sysid_primary.setter
    def sysid_primary(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sysid_primary' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sysid_primary' field must be an unsigned integer in [0, 255]"
        self._sysid_primary = value

    @builtins.property
    def compid_primary(self):
        """Message field 'compid_primary'."""
        return self._compid_primary

    @compid_primary.setter
    def compid_primary(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compid_primary' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compid_primary' field must be an unsigned integer in [0, 255]"
        self._compid_primary = value

    @builtins.property
    def sysid_secondary(self):
        """Message field 'sysid_secondary'."""
        return self._sysid_secondary

    @sysid_secondary.setter
    def sysid_secondary(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sysid_secondary' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sysid_secondary' field must be an unsigned integer in [0, 255]"
        self._sysid_secondary = value

    @builtins.property
    def compid_secondary(self):
        """Message field 'compid_secondary'."""
        return self._compid_secondary

    @compid_secondary.setter
    def compid_secondary(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compid_secondary' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compid_secondary' field must be an unsigned integer in [0, 255]"
        self._compid_secondary = value
