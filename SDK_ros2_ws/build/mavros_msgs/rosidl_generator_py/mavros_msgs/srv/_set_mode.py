# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/SetMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetMode_Request(type):
    """Metaclass of message 'SetMode_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAV_MODE_PREFLIGHT': 0,
        'MAV_MODE_STABILIZE_DISARMED': 80,
        'MAV_MODE_STABILIZE_ARMED': 208,
        'MAV_MODE_MANUAL_DISARMED': 64,
        'MAV_MODE_MANUAL_ARMED': 192,
        'MAV_MODE_GUIDED_DISARMED': 88,
        'MAV_MODE_GUIDED_ARMED': 216,
        'MAV_MODE_AUTO_DISARMED': 92,
        'MAV_MODE_AUTO_ARMED': 220,
        'MAV_MODE_TEST_DISARMED': 66,
        'MAV_MODE_TEST_ARMED': 194,
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
                'mavros_msgs.srv.SetMode_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_mode__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_mode__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_mode__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_mode__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_mode__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAV_MODE_PREFLIGHT': cls.__constants['MAV_MODE_PREFLIGHT'],
            'MAV_MODE_STABILIZE_DISARMED': cls.__constants['MAV_MODE_STABILIZE_DISARMED'],
            'MAV_MODE_STABILIZE_ARMED': cls.__constants['MAV_MODE_STABILIZE_ARMED'],
            'MAV_MODE_MANUAL_DISARMED': cls.__constants['MAV_MODE_MANUAL_DISARMED'],
            'MAV_MODE_MANUAL_ARMED': cls.__constants['MAV_MODE_MANUAL_ARMED'],
            'MAV_MODE_GUIDED_DISARMED': cls.__constants['MAV_MODE_GUIDED_DISARMED'],
            'MAV_MODE_GUIDED_ARMED': cls.__constants['MAV_MODE_GUIDED_ARMED'],
            'MAV_MODE_AUTO_DISARMED': cls.__constants['MAV_MODE_AUTO_DISARMED'],
            'MAV_MODE_AUTO_ARMED': cls.__constants['MAV_MODE_AUTO_ARMED'],
            'MAV_MODE_TEST_DISARMED': cls.__constants['MAV_MODE_TEST_DISARMED'],
            'MAV_MODE_TEST_ARMED': cls.__constants['MAV_MODE_TEST_ARMED'],
        }

    @property
    def MAV_MODE_PREFLIGHT(self):
        """Message constant 'MAV_MODE_PREFLIGHT'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_PREFLIGHT']

    @property
    def MAV_MODE_STABILIZE_DISARMED(self):
        """Message constant 'MAV_MODE_STABILIZE_DISARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_STABILIZE_DISARMED']

    @property
    def MAV_MODE_STABILIZE_ARMED(self):
        """Message constant 'MAV_MODE_STABILIZE_ARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_STABILIZE_ARMED']

    @property
    def MAV_MODE_MANUAL_DISARMED(self):
        """Message constant 'MAV_MODE_MANUAL_DISARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_MANUAL_DISARMED']

    @property
    def MAV_MODE_MANUAL_ARMED(self):
        """Message constant 'MAV_MODE_MANUAL_ARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_MANUAL_ARMED']

    @property
    def MAV_MODE_GUIDED_DISARMED(self):
        """Message constant 'MAV_MODE_GUIDED_DISARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_GUIDED_DISARMED']

    @property
    def MAV_MODE_GUIDED_ARMED(self):
        """Message constant 'MAV_MODE_GUIDED_ARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_GUIDED_ARMED']

    @property
    def MAV_MODE_AUTO_DISARMED(self):
        """Message constant 'MAV_MODE_AUTO_DISARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_AUTO_DISARMED']

    @property
    def MAV_MODE_AUTO_ARMED(self):
        """Message constant 'MAV_MODE_AUTO_ARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_AUTO_ARMED']

    @property
    def MAV_MODE_TEST_DISARMED(self):
        """Message constant 'MAV_MODE_TEST_DISARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_TEST_DISARMED']

    @property
    def MAV_MODE_TEST_ARMED(self):
        """Message constant 'MAV_MODE_TEST_ARMED'."""
        return Metaclass_SetMode_Request.__constants['MAV_MODE_TEST_ARMED']


class SetMode_Request(metaclass=Metaclass_SetMode_Request):
    """
    Message class 'SetMode_Request'.

    Constants:
      MAV_MODE_PREFLIGHT
      MAV_MODE_STABILIZE_DISARMED
      MAV_MODE_STABILIZE_ARMED
      MAV_MODE_MANUAL_DISARMED
      MAV_MODE_MANUAL_ARMED
      MAV_MODE_GUIDED_DISARMED
      MAV_MODE_GUIDED_ARMED
      MAV_MODE_AUTO_DISARMED
      MAV_MODE_AUTO_ARMED
      MAV_MODE_TEST_DISARMED
      MAV_MODE_TEST_ARMED
    """

    __slots__ = [
        '_base_mode',
        '_custom_mode',
    ]

    _fields_and_field_types = {
        'base_mode': 'uint8',
        'custom_mode': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.base_mode = kwargs.get('base_mode', int())
        self.custom_mode = kwargs.get('custom_mode', str())

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
        if self.base_mode != other.base_mode:
            return False
        if self.custom_mode != other.custom_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def base_mode(self):
        """Message field 'base_mode'."""
        return self._base_mode

    @base_mode.setter
    def base_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'base_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'base_mode' field must be an unsigned integer in [0, 255]"
        self._base_mode = value

    @builtins.property
    def custom_mode(self):
        """Message field 'custom_mode'."""
        return self._custom_mode

    @custom_mode.setter
    def custom_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'custom_mode' field must be of type 'str'"
        self._custom_mode = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetMode_Response(type):
    """Metaclass of message 'SetMode_Response'."""

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
                'mavros_msgs.srv.SetMode_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_mode__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_mode__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_mode__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_mode__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_mode__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetMode_Response(metaclass=Metaclass_SetMode_Response):
    """Message class 'SetMode_Response'."""

    __slots__ = [
        '_mode_sent',
    ]

    _fields_and_field_types = {
        'mode_sent': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode_sent = kwargs.get('mode_sent', bool())

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
        if self.mode_sent != other.mode_sent:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode_sent(self):
        """Message field 'mode_sent'."""
        return self._mode_sent

    @mode_sent.setter
    def mode_sent(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'mode_sent' field must be of type 'bool'"
        self._mode_sent = value


class Metaclass_SetMode(type):
    """Metaclass of service 'SetMode'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mavros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mavros_msgs.srv.SetMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_mode

            from mavros_msgs.srv import _set_mode
            if _set_mode.Metaclass_SetMode_Request._TYPE_SUPPORT is None:
                _set_mode.Metaclass_SetMode_Request.__import_type_support__()
            if _set_mode.Metaclass_SetMode_Response._TYPE_SUPPORT is None:
                _set_mode.Metaclass_SetMode_Response.__import_type_support__()


class SetMode(metaclass=Metaclass_SetMode):
    from mavros_msgs.srv._set_mode import SetMode_Request as Request
    from mavros_msgs.srv._set_mode import SetMode_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
