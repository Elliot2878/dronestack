# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/FileOpen.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FileOpen_Request(type):
    """Metaclass of message 'FileOpen_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_READ': 0,
        'MODE_WRITE': 1,
        'MODE_CREATE': 2,
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
                'mavros_msgs.srv.FileOpen_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__file_open__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__file_open__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__file_open__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__file_open__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__file_open__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_READ': cls.__constants['MODE_READ'],
            'MODE_WRITE': cls.__constants['MODE_WRITE'],
            'MODE_CREATE': cls.__constants['MODE_CREATE'],
        }

    @property
    def MODE_READ(self):
        """Message constant 'MODE_READ'."""
        return Metaclass_FileOpen_Request.__constants['MODE_READ']

    @property
    def MODE_WRITE(self):
        """Message constant 'MODE_WRITE'."""
        return Metaclass_FileOpen_Request.__constants['MODE_WRITE']

    @property
    def MODE_CREATE(self):
        """Message constant 'MODE_CREATE'."""
        return Metaclass_FileOpen_Request.__constants['MODE_CREATE']


class FileOpen_Request(metaclass=Metaclass_FileOpen_Request):
    """
    Message class 'FileOpen_Request'.

    Constants:
      MODE_READ
      MODE_WRITE
      MODE_CREATE
    """

    __slots__ = [
        '_file_path',
        '_mode',
    ]

    _fields_and_field_types = {
        'file_path': 'string',
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.file_path = kwargs.get('file_path', str())
        self.mode = kwargs.get('mode', int())

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
        if self.file_path != other.file_path:
            return False
        if self.mode != other.mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def file_path(self):
        """Message field 'file_path'."""
        return self._file_path

    @file_path.setter
    def file_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'file_path' field must be of type 'str'"
        self._file_path = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_FileOpen_Response(type):
    """Metaclass of message 'FileOpen_Response'."""

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
                'mavros_msgs.srv.FileOpen_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__file_open__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__file_open__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__file_open__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__file_open__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__file_open__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FileOpen_Response(metaclass=Metaclass_FileOpen_Response):
    """Message class 'FileOpen_Response'."""

    __slots__ = [
        '_size',
        '_success',
        '_r_errno',
    ]

    _fields_and_field_types = {
        'size': 'uint32',
        'success': 'boolean',
        'r_errno': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.size = kwargs.get('size', int())
        self.success = kwargs.get('success', bool())
        self.r_errno = kwargs.get('r_errno', int())

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
        if self.size != other.size:
            return False
        if self.success != other.success:
            return False
        if self.r_errno != other.r_errno:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def size(self):
        """Message field 'size'."""
        return self._size

    @size.setter
    def size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'size' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'size' field must be an unsigned integer in [0, 4294967295]"
        self._size = value

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
    def r_errno(self):
        """Message field 'r_errno'."""
        return self._r_errno

    @r_errno.setter
    def r_errno(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'r_errno' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'r_errno' field must be an integer in [-2147483648, 2147483647]"
        self._r_errno = value


class Metaclass_FileOpen(type):
    """Metaclass of service 'FileOpen'."""

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
                'mavros_msgs.srv.FileOpen')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__file_open

            from mavros_msgs.srv import _file_open
            if _file_open.Metaclass_FileOpen_Request._TYPE_SUPPORT is None:
                _file_open.Metaclass_FileOpen_Request.__import_type_support__()
            if _file_open.Metaclass_FileOpen_Response._TYPE_SUPPORT is None:
                _file_open.Metaclass_FileOpen_Response.__import_type_support__()


class FileOpen(metaclass=Metaclass_FileOpen):
    from mavros_msgs.srv._file_open import FileOpen_Request as Request
    from mavros_msgs.srv._file_open import FileOpen_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
