# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/ParamSet.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ParamSet_Request(type):
    """Metaclass of message 'ParamSet_Request'."""

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
                'mavros_msgs.srv.ParamSet_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__param_set__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__param_set__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__param_set__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__param_set__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__param_set__request

            from mavros_msgs.msg import ParamValue
            if ParamValue.__class__._TYPE_SUPPORT is None:
                ParamValue.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ParamSet_Request(metaclass=Metaclass_ParamSet_Request):
    """Message class 'ParamSet_Request'."""

    __slots__ = [
        '_param_id',
        '_value',
    ]

    _fields_and_field_types = {
        'param_id': 'string',
        'value': 'mavros_msgs/ParamValue',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'ParamValue'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.param_id = kwargs.get('param_id', str())
        from mavros_msgs.msg import ParamValue
        self.value = kwargs.get('value', ParamValue())

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
        if self.param_id != other.param_id:
            return False
        if self.value != other.value:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def param_id(self):
        """Message field 'param_id'."""
        return self._param_id

    @param_id.setter
    def param_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'param_id' field must be of type 'str'"
        self._param_id = value

    @builtins.property
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if __debug__:
            from mavros_msgs.msg import ParamValue
            assert \
                isinstance(value, ParamValue), \
                "The 'value' field must be a sub message of type 'ParamValue'"
        self._value = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ParamSet_Response(type):
    """Metaclass of message 'ParamSet_Response'."""

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
                'mavros_msgs.srv.ParamSet_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__param_set__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__param_set__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__param_set__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__param_set__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__param_set__response

            from mavros_msgs.msg import ParamValue
            if ParamValue.__class__._TYPE_SUPPORT is None:
                ParamValue.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ParamSet_Response(metaclass=Metaclass_ParamSet_Response):
    """Message class 'ParamSet_Response'."""

    __slots__ = [
        '_success',
        '_value',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'value': 'mavros_msgs/ParamValue',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'ParamValue'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        from mavros_msgs.msg import ParamValue
        self.value = kwargs.get('value', ParamValue())

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
        if self.value != other.value:
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
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if __debug__:
            from mavros_msgs.msg import ParamValue
            assert \
                isinstance(value, ParamValue), \
                "The 'value' field must be a sub message of type 'ParamValue'"
        self._value = value


class Metaclass_ParamSet(type):
    """Metaclass of service 'ParamSet'."""

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
                'mavros_msgs.srv.ParamSet')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__param_set

            from mavros_msgs.srv import _param_set
            if _param_set.Metaclass_ParamSet_Request._TYPE_SUPPORT is None:
                _param_set.Metaclass_ParamSet_Request.__import_type_support__()
            if _param_set.Metaclass_ParamSet_Response._TYPE_SUPPORT is None:
                _param_set.Metaclass_ParamSet_Response.__import_type_support__()


class ParamSet(metaclass=Metaclass_ParamSet):
    from mavros_msgs.srv._param_set import ParamSet_Request as Request
    from mavros_msgs.srv._param_set import ParamSet_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
