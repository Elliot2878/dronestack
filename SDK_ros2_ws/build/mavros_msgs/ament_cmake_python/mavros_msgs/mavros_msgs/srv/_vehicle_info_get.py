# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/VehicleInfoGet.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VehicleInfoGet_Request(type):
    """Metaclass of message 'VehicleInfoGet_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GET_MY_SYSID': 0,
        'GET_MY_COMPID': 0,
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
                'mavros_msgs.srv.VehicleInfoGet_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__vehicle_info_get__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__vehicle_info_get__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__vehicle_info_get__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__vehicle_info_get__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__vehicle_info_get__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GET_MY_SYSID': cls.__constants['GET_MY_SYSID'],
            'GET_MY_COMPID': cls.__constants['GET_MY_COMPID'],
        }

    @property
    def GET_MY_SYSID(self):
        """Message constant 'GET_MY_SYSID'."""
        return Metaclass_VehicleInfoGet_Request.__constants['GET_MY_SYSID']

    @property
    def GET_MY_COMPID(self):
        """Message constant 'GET_MY_COMPID'."""
        return Metaclass_VehicleInfoGet_Request.__constants['GET_MY_COMPID']


class VehicleInfoGet_Request(metaclass=Metaclass_VehicleInfoGet_Request):
    """
    Message class 'VehicleInfoGet_Request'.

    Constants:
      GET_MY_SYSID
      GET_MY_COMPID
    """

    __slots__ = [
        '_sysid',
        '_compid',
        '_get_all',
    ]

    _fields_and_field_types = {
        'sysid': 'uint8',
        'compid': 'uint8',
        'get_all': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sysid = kwargs.get('sysid', int())
        self.compid = kwargs.get('compid', int())
        self.get_all = kwargs.get('get_all', bool())

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
        if self.sysid != other.sysid:
            return False
        if self.compid != other.compid:
            return False
        if self.get_all != other.get_all:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sysid(self):
        """Message field 'sysid'."""
        return self._sysid

    @sysid.setter
    def sysid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sysid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sysid' field must be an unsigned integer in [0, 255]"
        self._sysid = value

    @builtins.property
    def compid(self):
        """Message field 'compid'."""
        return self._compid

    @compid.setter
    def compid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compid' field must be an unsigned integer in [0, 255]"
        self._compid = value

    @builtins.property
    def get_all(self):
        """Message field 'get_all'."""
        return self._get_all

    @get_all.setter
    def get_all(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'get_all' field must be of type 'bool'"
        self._get_all = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_VehicleInfoGet_Response(type):
    """Metaclass of message 'VehicleInfoGet_Response'."""

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
                'mavros_msgs.srv.VehicleInfoGet_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__vehicle_info_get__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__vehicle_info_get__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__vehicle_info_get__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__vehicle_info_get__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__vehicle_info_get__response

            from mavros_msgs.msg import VehicleInfo
            if VehicleInfo.__class__._TYPE_SUPPORT is None:
                VehicleInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VehicleInfoGet_Response(metaclass=Metaclass_VehicleInfoGet_Response):
    """Message class 'VehicleInfoGet_Response'."""

    __slots__ = [
        '_success',
        '_vehicles',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'vehicles': 'sequence<mavros_msgs/VehicleInfo>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'VehicleInfo')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.vehicles = kwargs.get('vehicles', [])

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
        if self.vehicles != other.vehicles:
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
    def vehicles(self):
        """Message field 'vehicles'."""
        return self._vehicles

    @vehicles.setter
    def vehicles(self, value):
        if __debug__:
            from mavros_msgs.msg import VehicleInfo
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
                 all(isinstance(v, VehicleInfo) for v in value) and
                 True), \
                "The 'vehicles' field must be a set or sequence and each value of type 'VehicleInfo'"
        self._vehicles = value


class Metaclass_VehicleInfoGet(type):
    """Metaclass of service 'VehicleInfoGet'."""

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
                'mavros_msgs.srv.VehicleInfoGet')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__vehicle_info_get

            from mavros_msgs.srv import _vehicle_info_get
            if _vehicle_info_get.Metaclass_VehicleInfoGet_Request._TYPE_SUPPORT is None:
                _vehicle_info_get.Metaclass_VehicleInfoGet_Request.__import_type_support__()
            if _vehicle_info_get.Metaclass_VehicleInfoGet_Response._TYPE_SUPPORT is None:
                _vehicle_info_get.Metaclass_VehicleInfoGet_Response.__import_type_support__()


class VehicleInfoGet(metaclass=Metaclass_VehicleInfoGet):
    from mavros_msgs.srv._vehicle_info_get import VehicleInfoGet_Request as Request
    from mavros_msgs.srv._vehicle_info_get import VehicleInfoGet_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
