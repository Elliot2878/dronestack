# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/GimbalManagerConfigure.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalManagerConfigure_Request(type):
    """Metaclass of message 'GimbalManagerConfigure_Request'."""

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
                'mavros_msgs.srv.GimbalManagerConfigure_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gimbal_manager_configure__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gimbal_manager_configure__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gimbal_manager_configure__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gimbal_manager_configure__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gimbal_manager_configure__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GimbalManagerConfigure_Request(metaclass=Metaclass_GimbalManagerConfigure_Request):
    """Message class 'GimbalManagerConfigure_Request'."""

    __slots__ = [
        '_sysid_primary',
        '_compid_primary',
        '_sysid_secondary',
        '_compid_secondary',
        '_gimbal_device_id',
    ]

    _fields_and_field_types = {
        'sysid_primary': 'int16',
        'compid_primary': 'int16',
        'sysid_secondary': 'int16',
        'compid_secondary': 'int16',
        'gimbal_device_id': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sysid_primary = kwargs.get('sysid_primary', int())
        self.compid_primary = kwargs.get('compid_primary', int())
        self.sysid_secondary = kwargs.get('sysid_secondary', int())
        self.compid_secondary = kwargs.get('compid_secondary', int())
        self.gimbal_device_id = kwargs.get('gimbal_device_id', int())

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
        if self.sysid_primary != other.sysid_primary:
            return False
        if self.compid_primary != other.compid_primary:
            return False
        if self.sysid_secondary != other.sysid_secondary:
            return False
        if self.compid_secondary != other.compid_secondary:
            return False
        if self.gimbal_device_id != other.gimbal_device_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
            assert value >= -32768 and value < 32768, \
                "The 'sysid_primary' field must be an integer in [-32768, 32767]"
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
            assert value >= -32768 and value < 32768, \
                "The 'compid_primary' field must be an integer in [-32768, 32767]"
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
            assert value >= -32768 and value < 32768, \
                "The 'sysid_secondary' field must be an integer in [-32768, 32767]"
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
            assert value >= -32768 and value < 32768, \
                "The 'compid_secondary' field must be an integer in [-32768, 32767]"
        self._compid_secondary = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GimbalManagerConfigure_Response(type):
    """Metaclass of message 'GimbalManagerConfigure_Response'."""

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
                'mavros_msgs.srv.GimbalManagerConfigure_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gimbal_manager_configure__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gimbal_manager_configure__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gimbal_manager_configure__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gimbal_manager_configure__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gimbal_manager_configure__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GimbalManagerConfigure_Response(metaclass=Metaclass_GimbalManagerConfigure_Response):
    """Message class 'GimbalManagerConfigure_Response'."""

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


class Metaclass_GimbalManagerConfigure(type):
    """Metaclass of service 'GimbalManagerConfigure'."""

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
                'mavros_msgs.srv.GimbalManagerConfigure')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__gimbal_manager_configure

            from mavros_msgs.srv import _gimbal_manager_configure
            if _gimbal_manager_configure.Metaclass_GimbalManagerConfigure_Request._TYPE_SUPPORT is None:
                _gimbal_manager_configure.Metaclass_GimbalManagerConfigure_Request.__import_type_support__()
            if _gimbal_manager_configure.Metaclass_GimbalManagerConfigure_Response._TYPE_SUPPORT is None:
                _gimbal_manager_configure.Metaclass_GimbalManagerConfigure_Response.__import_type_support__()


class GimbalManagerConfigure(metaclass=Metaclass_GimbalManagerConfigure):
    from mavros_msgs.srv._gimbal_manager_configure import GimbalManagerConfigure_Request as Request
    from mavros_msgs.srv._gimbal_manager_configure import GimbalManagerConfigure_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
