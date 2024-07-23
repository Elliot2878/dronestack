# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/ParamPull.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ParamPull_Request(type):
    """Metaclass of message 'ParamPull_Request'."""

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
                'mavros_msgs.srv.ParamPull_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__param_pull__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__param_pull__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__param_pull__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__param_pull__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__param_pull__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ParamPull_Request(metaclass=Metaclass_ParamPull_Request):
    """Message class 'ParamPull_Request'."""

    __slots__ = [
        '_force_pull',
    ]

    _fields_and_field_types = {
        'force_pull': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.force_pull = kwargs.get('force_pull', bool())

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
        if self.force_pull != other.force_pull:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def force_pull(self):
        """Message field 'force_pull'."""
        return self._force_pull

    @force_pull.setter
    def force_pull(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'force_pull' field must be of type 'bool'"
        self._force_pull = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ParamPull_Response(type):
    """Metaclass of message 'ParamPull_Response'."""

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
                'mavros_msgs.srv.ParamPull_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__param_pull__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__param_pull__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__param_pull__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__param_pull__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__param_pull__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ParamPull_Response(metaclass=Metaclass_ParamPull_Response):
    """Message class 'ParamPull_Response'."""

    __slots__ = [
        '_success',
        '_param_received',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'param_received': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.param_received = kwargs.get('param_received', int())

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
        if self.param_received != other.param_received:
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
    def param_received(self):
        """Message field 'param_received'."""
        return self._param_received

    @param_received.setter
    def param_received(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param_received' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'param_received' field must be an unsigned integer in [0, 4294967295]"
        self._param_received = value


class Metaclass_ParamPull(type):
    """Metaclass of service 'ParamPull'."""

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
                'mavros_msgs.srv.ParamPull')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__param_pull

            from mavros_msgs.srv import _param_pull
            if _param_pull.Metaclass_ParamPull_Request._TYPE_SUPPORT is None:
                _param_pull.Metaclass_ParamPull_Request.__import_type_support__()
            if _param_pull.Metaclass_ParamPull_Response._TYPE_SUPPORT is None:
                _param_pull.Metaclass_ParamPull_Response.__import_type_support__()


class ParamPull(metaclass=Metaclass_ParamPull):
    from mavros_msgs.srv._param_pull import ParamPull_Request as Request
    from mavros_msgs.srv._param_pull import ParamPull_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
