# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/CommandTriggerControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CommandTriggerControl_Request(type):
    """Metaclass of message 'CommandTriggerControl_Request'."""

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
                'mavros_msgs.srv.CommandTriggerControl_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__command_trigger_control__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__command_trigger_control__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__command_trigger_control__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__command_trigger_control__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__command_trigger_control__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommandTriggerControl_Request(metaclass=Metaclass_CommandTriggerControl_Request):
    """Message class 'CommandTriggerControl_Request'."""

    __slots__ = [
        '_trigger_enable',
        '_sequence_reset',
        '_trigger_pause',
    ]

    _fields_and_field_types = {
        'trigger_enable': 'boolean',
        'sequence_reset': 'boolean',
        'trigger_pause': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.trigger_enable = kwargs.get('trigger_enable', bool())
        self.sequence_reset = kwargs.get('sequence_reset', bool())
        self.trigger_pause = kwargs.get('trigger_pause', bool())

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
        if self.trigger_enable != other.trigger_enable:
            return False
        if self.sequence_reset != other.sequence_reset:
            return False
        if self.trigger_pause != other.trigger_pause:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def trigger_enable(self):
        """Message field 'trigger_enable'."""
        return self._trigger_enable

    @trigger_enable.setter
    def trigger_enable(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trigger_enable' field must be of type 'bool'"
        self._trigger_enable = value

    @builtins.property
    def sequence_reset(self):
        """Message field 'sequence_reset'."""
        return self._sequence_reset

    @sequence_reset.setter
    def sequence_reset(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sequence_reset' field must be of type 'bool'"
        self._sequence_reset = value

    @builtins.property
    def trigger_pause(self):
        """Message field 'trigger_pause'."""
        return self._trigger_pause

    @trigger_pause.setter
    def trigger_pause(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trigger_pause' field must be of type 'bool'"
        self._trigger_pause = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CommandTriggerControl_Response(type):
    """Metaclass of message 'CommandTriggerControl_Response'."""

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
                'mavros_msgs.srv.CommandTriggerControl_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__command_trigger_control__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__command_trigger_control__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__command_trigger_control__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__command_trigger_control__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__command_trigger_control__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommandTriggerControl_Response(metaclass=Metaclass_CommandTriggerControl_Response):
    """Message class 'CommandTriggerControl_Response'."""

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


class Metaclass_CommandTriggerControl(type):
    """Metaclass of service 'CommandTriggerControl'."""

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
                'mavros_msgs.srv.CommandTriggerControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__command_trigger_control

            from mavros_msgs.srv import _command_trigger_control
            if _command_trigger_control.Metaclass_CommandTriggerControl_Request._TYPE_SUPPORT is None:
                _command_trigger_control.Metaclass_CommandTriggerControl_Request.__import_type_support__()
            if _command_trigger_control.Metaclass_CommandTriggerControl_Response._TYPE_SUPPORT is None:
                _command_trigger_control.Metaclass_CommandTriggerControl_Response.__import_type_support__()


class CommandTriggerControl(metaclass=Metaclass_CommandTriggerControl):
    from mavros_msgs.srv._command_trigger_control import CommandTriggerControl_Request as Request
    from mavros_msgs.srv._command_trigger_control import CommandTriggerControl_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
