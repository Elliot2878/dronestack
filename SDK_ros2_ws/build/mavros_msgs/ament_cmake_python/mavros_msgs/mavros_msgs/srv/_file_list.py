# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/FileList.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FileList_Request(type):
    """Metaclass of message 'FileList_Request'."""

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
                'mavros_msgs.srv.FileList_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__file_list__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__file_list__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__file_list__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__file_list__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__file_list__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FileList_Request(metaclass=Metaclass_FileList_Request):
    """Message class 'FileList_Request'."""

    __slots__ = [
        '_dir_path',
    ]

    _fields_and_field_types = {
        'dir_path': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.dir_path = kwargs.get('dir_path', str())

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
        if self.dir_path != other.dir_path:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def dir_path(self):
        """Message field 'dir_path'."""
        return self._dir_path

    @dir_path.setter
    def dir_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'dir_path' field must be of type 'str'"
        self._dir_path = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_FileList_Response(type):
    """Metaclass of message 'FileList_Response'."""

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
                'mavros_msgs.srv.FileList_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__file_list__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__file_list__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__file_list__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__file_list__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__file_list__response

            from mavros_msgs.msg import FileEntry
            if FileEntry.__class__._TYPE_SUPPORT is None:
                FileEntry.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FileList_Response(metaclass=Metaclass_FileList_Response):
    """Message class 'FileList_Response'."""

    __slots__ = [
        '_list',
        '_success',
        '_r_errno',
    ]

    _fields_and_field_types = {
        'list': 'sequence<mavros_msgs/FileEntry>',
        'success': 'boolean',
        'r_errno': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'FileEntry')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.list = kwargs.get('list', [])
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
        if self.list != other.list:
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

    @builtins.property  # noqa: A003
    def list(self):  # noqa: A003
        """Message field 'list'."""
        return self._list

    @list.setter  # noqa: A003
    def list(self, value):  # noqa: A003
        if __debug__:
            from mavros_msgs.msg import FileEntry
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, FileEntry) for v in value) and
                 True), \
                "The 'list' field must be a set or sequence and each value of type 'FileEntry'"
        self._list = value

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


class Metaclass_FileList(type):
    """Metaclass of service 'FileList'."""

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
                'mavros_msgs.srv.FileList')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__file_list

            from mavros_msgs.srv import _file_list
            if _file_list.Metaclass_FileList_Request._TYPE_SUPPORT is None:
                _file_list.Metaclass_FileList_Request.__import_type_support__()
            if _file_list.Metaclass_FileList_Response._TYPE_SUPPORT is None:
                _file_list.Metaclass_FileList_Response.__import_type_support__()


class FileList(metaclass=Metaclass_FileList):
    from mavros_msgs.srv._file_list import FileList_Request as Request
    from mavros_msgs.srv._file_list import FileList_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
