# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/SysStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SysStatus(type):
    """Metaclass of message 'SysStatus'."""

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
                'mavros_msgs.msg.SysStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sys_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sys_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sys_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sys_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sys_status

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


class SysStatus(metaclass=Metaclass_SysStatus):
    """Message class 'SysStatus'."""

    __slots__ = [
        '_header',
        '_sensors_present',
        '_sensors_enabled',
        '_sensors_health',
        '_load',
        '_voltage_battery',
        '_current_battery',
        '_battery_remaining',
        '_drop_rate_comm',
        '_errors_comm',
        '_errors_count1',
        '_errors_count2',
        '_errors_count3',
        '_errors_count4',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'sensors_present': 'uint32',
        'sensors_enabled': 'uint32',
        'sensors_health': 'uint32',
        'load': 'uint16',
        'voltage_battery': 'uint16',
        'current_battery': 'int16',
        'battery_remaining': 'int8',
        'drop_rate_comm': 'uint16',
        'errors_comm': 'uint16',
        'errors_count1': 'uint16',
        'errors_count2': 'uint16',
        'errors_count3': 'uint16',
        'errors_count4': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.sensors_present = kwargs.get('sensors_present', int())
        self.sensors_enabled = kwargs.get('sensors_enabled', int())
        self.sensors_health = kwargs.get('sensors_health', int())
        self.load = kwargs.get('load', int())
        self.voltage_battery = kwargs.get('voltage_battery', int())
        self.current_battery = kwargs.get('current_battery', int())
        self.battery_remaining = kwargs.get('battery_remaining', int())
        self.drop_rate_comm = kwargs.get('drop_rate_comm', int())
        self.errors_comm = kwargs.get('errors_comm', int())
        self.errors_count1 = kwargs.get('errors_count1', int())
        self.errors_count2 = kwargs.get('errors_count2', int())
        self.errors_count3 = kwargs.get('errors_count3', int())
        self.errors_count4 = kwargs.get('errors_count4', int())

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
        if self.sensors_present != other.sensors_present:
            return False
        if self.sensors_enabled != other.sensors_enabled:
            return False
        if self.sensors_health != other.sensors_health:
            return False
        if self.load != other.load:
            return False
        if self.voltage_battery != other.voltage_battery:
            return False
        if self.current_battery != other.current_battery:
            return False
        if self.battery_remaining != other.battery_remaining:
            return False
        if self.drop_rate_comm != other.drop_rate_comm:
            return False
        if self.errors_comm != other.errors_comm:
            return False
        if self.errors_count1 != other.errors_count1:
            return False
        if self.errors_count2 != other.errors_count2:
            return False
        if self.errors_count3 != other.errors_count3:
            return False
        if self.errors_count4 != other.errors_count4:
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
    def sensors_present(self):
        """Message field 'sensors_present'."""
        return self._sensors_present

    @sensors_present.setter
    def sensors_present(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensors_present' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'sensors_present' field must be an unsigned integer in [0, 4294967295]"
        self._sensors_present = value

    @builtins.property
    def sensors_enabled(self):
        """Message field 'sensors_enabled'."""
        return self._sensors_enabled

    @sensors_enabled.setter
    def sensors_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensors_enabled' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'sensors_enabled' field must be an unsigned integer in [0, 4294967295]"
        self._sensors_enabled = value

    @builtins.property
    def sensors_health(self):
        """Message field 'sensors_health'."""
        return self._sensors_health

    @sensors_health.setter
    def sensors_health(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensors_health' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'sensors_health' field must be an unsigned integer in [0, 4294967295]"
        self._sensors_health = value

    @builtins.property
    def load(self):
        """Message field 'load'."""
        return self._load

    @load.setter
    def load(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'load' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'load' field must be an unsigned integer in [0, 65535]"
        self._load = value

    @builtins.property
    def voltage_battery(self):
        """Message field 'voltage_battery'."""
        return self._voltage_battery

    @voltage_battery.setter
    def voltage_battery(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'voltage_battery' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'voltage_battery' field must be an unsigned integer in [0, 65535]"
        self._voltage_battery = value

    @builtins.property
    def current_battery(self):
        """Message field 'current_battery'."""
        return self._current_battery

    @current_battery.setter
    def current_battery(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_battery' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'current_battery' field must be an integer in [-32768, 32767]"
        self._current_battery = value

    @builtins.property
    def battery_remaining(self):
        """Message field 'battery_remaining'."""
        return self._battery_remaining

    @battery_remaining.setter
    def battery_remaining(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'battery_remaining' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'battery_remaining' field must be an integer in [-128, 127]"
        self._battery_remaining = value

    @builtins.property
    def drop_rate_comm(self):
        """Message field 'drop_rate_comm'."""
        return self._drop_rate_comm

    @drop_rate_comm.setter
    def drop_rate_comm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'drop_rate_comm' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'drop_rate_comm' field must be an unsigned integer in [0, 65535]"
        self._drop_rate_comm = value

    @builtins.property
    def errors_comm(self):
        """Message field 'errors_comm'."""
        return self._errors_comm

    @errors_comm.setter
    def errors_comm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'errors_comm' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'errors_comm' field must be an unsigned integer in [0, 65535]"
        self._errors_comm = value

    @builtins.property
    def errors_count1(self):
        """Message field 'errors_count1'."""
        return self._errors_count1

    @errors_count1.setter
    def errors_count1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'errors_count1' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'errors_count1' field must be an unsigned integer in [0, 65535]"
        self._errors_count1 = value

    @builtins.property
    def errors_count2(self):
        """Message field 'errors_count2'."""
        return self._errors_count2

    @errors_count2.setter
    def errors_count2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'errors_count2' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'errors_count2' field must be an unsigned integer in [0, 65535]"
        self._errors_count2 = value

    @builtins.property
    def errors_count3(self):
        """Message field 'errors_count3'."""
        return self._errors_count3

    @errors_count3.setter
    def errors_count3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'errors_count3' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'errors_count3' field must be an unsigned integer in [0, 65535]"
        self._errors_count3 = value

    @builtins.property
    def errors_count4(self):
        """Message field 'errors_count4'."""
        return self._errors_count4

    @errors_count4.setter
    def errors_count4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'errors_count4' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'errors_count4' field must be an unsigned integer in [0, 65535]"
        self._errors_count4 = value
