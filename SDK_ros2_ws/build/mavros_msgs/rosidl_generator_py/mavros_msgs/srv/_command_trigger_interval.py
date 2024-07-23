# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/CommandTriggerInterval.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CommandTriggerInterval_Request(type):
    """Metaclass of message 'CommandTriggerInterval_Request'."""

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
                'mavros_msgs.srv.CommandTriggerInterval_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__command_trigger_interval__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__command_trigger_interval__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__command_trigger_interval__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__command_trigger_interval__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__command_trigger_interval__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommandTriggerInterval_Request(metaclass=Metaclass_CommandTriggerInterval_Request):
    """Message class 'CommandTriggerInterval_Request'."""

    __slots__ = [
        '_cycle_time',
        '_integration_time',
    ]

    _fields_and_field_types = {
        'cycle_time': 'float',
        'integration_time': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cycle_time = kwargs.get('cycle_time', float())
        self.integration_time = kwargs.get('integration_time', float())

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
        if self.cycle_time != other.cycle_time:
            return False
        if self.integration_time != other.integration_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cycle_time(self):
        """Message field 'cycle_time'."""
        return self._cycle_time

    @cycle_time.setter
    def cycle_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cycle_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cycle_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cycle_time = value

    @builtins.property
    def integration_time(self):
        """Message field 'integration_time'."""
        return self._integration_time

    @integration_time.setter
    def integration_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integration_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'integration_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._integration_time = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CommandTriggerInterval_Response(type):
    """Metaclass of message 'CommandTriggerInterval_Response'."""

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
                'mavros_msgs.srv.CommandTriggerInterval_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__command_trigger_interval__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__command_trigger_interval__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__command_trigger_interval__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__command_trigger_interval__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__command_trigger_interval__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommandTriggerInterval_Response(metaclass=Metaclass_CommandTriggerInterval_Response):
    """Message class 'CommandTriggerInterval_Response'."""

    __slots__ = [
        '_success',
        '_result',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'result': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.result = kwargs.get('result', int())

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
        if self.success != other.success:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'result' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'result' field must be an unsigned integer in [0, 255]"
        self._result = value


class Metaclass_CommandTriggerInterval(type):
    """Metaclass of service 'CommandTriggerInterval'."""

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
                'mavros_msgs.srv.CommandTriggerInterval')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__command_trigger_interval

            from mavros_msgs.srv import _command_trigger_interval
            if _command_trigger_interval.Metaclass_CommandTriggerInterval_Request._TYPE_SUPPORT is None:
                _command_trigger_interval.Metaclass_CommandTriggerInterval_Request.__import_type_support__()
            if _command_trigger_interval.Metaclass_CommandTriggerInterval_Response._TYPE_SUPPORT is None:
                _command_trigger_interval.Metaclass_CommandTriggerInterval_Response.__import_type_support__()


class CommandTriggerInterval(metaclass=Metaclass_CommandTriggerInterval):
    from mavros_msgs.srv._command_trigger_interval import CommandTriggerInterval_Request as Request
    from mavros_msgs.srv._command_trigger_interval import CommandTriggerInterval_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
