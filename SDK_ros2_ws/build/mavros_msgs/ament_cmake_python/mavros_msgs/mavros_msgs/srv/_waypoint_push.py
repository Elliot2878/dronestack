# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/WaypointPush.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WaypointPush_Request(type):
    """Metaclass of message 'WaypointPush_Request'."""

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
                'mavros_msgs.srv.WaypointPush_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__waypoint_push__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__waypoint_push__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__waypoint_push__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__waypoint_push__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__waypoint_push__request

            from mavros_msgs.msg import Waypoint
            if Waypoint.__class__._TYPE_SUPPORT is None:
                Waypoint.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WaypointPush_Request(metaclass=Metaclass_WaypointPush_Request):
    """Message class 'WaypointPush_Request'."""

    __slots__ = [
        '_start_index',
        '_waypoints',
    ]

    _fields_and_field_types = {
        'start_index': 'uint16',
        'waypoints': 'sequence<mavros_msgs/Waypoint>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'Waypoint')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start_index = kwargs.get('start_index', int())
        self.waypoints = kwargs.get('waypoints', [])

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
        if self.start_index != other.start_index:
            return False
        if self.waypoints != other.waypoints:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start_index(self):
        """Message field 'start_index'."""
        return self._start_index

    @start_index.setter
    def start_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'start_index' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'start_index' field must be an unsigned integer in [0, 65535]"
        self._start_index = value

    @builtins.property
    def waypoints(self):
        """Message field 'waypoints'."""
        return self._waypoints

    @waypoints.setter
    def waypoints(self, value):
        if __debug__:
            from mavros_msgs.msg import Waypoint
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
                 all(isinstance(v, Waypoint) for v in value) and
                 True), \
                "The 'waypoints' field must be a set or sequence and each value of type 'Waypoint'"
        self._waypoints = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_WaypointPush_Response(type):
    """Metaclass of message 'WaypointPush_Response'."""

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
                'mavros_msgs.srv.WaypointPush_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__waypoint_push__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__waypoint_push__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__waypoint_push__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__waypoint_push__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__waypoint_push__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WaypointPush_Response(metaclass=Metaclass_WaypointPush_Response):
    """Message class 'WaypointPush_Response'."""

    __slots__ = [
        '_success',
        '_wp_transfered',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'wp_transfered': 'uint32',
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
        self.wp_transfered = kwargs.get('wp_transfered', int())

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
        if self.wp_transfered != other.wp_transfered:
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
    def wp_transfered(self):
        """Message field 'wp_transfered'."""
        return self._wp_transfered

    @wp_transfered.setter
    def wp_transfered(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'wp_transfered' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'wp_transfered' field must be an unsigned integer in [0, 4294967295]"
        self._wp_transfered = value


class Metaclass_WaypointPush(type):
    """Metaclass of service 'WaypointPush'."""

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
                'mavros_msgs.srv.WaypointPush')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__waypoint_push

            from mavros_msgs.srv import _waypoint_push
            if _waypoint_push.Metaclass_WaypointPush_Request._TYPE_SUPPORT is None:
                _waypoint_push.Metaclass_WaypointPush_Request.__import_type_support__()
            if _waypoint_push.Metaclass_WaypointPush_Response._TYPE_SUPPORT is None:
                _waypoint_push.Metaclass_WaypointPush_Response.__import_type_support__()


class WaypointPush(metaclass=Metaclass_WaypointPush):
    from mavros_msgs.srv._waypoint_push import WaypointPush_Request as Request
    from mavros_msgs.srv._waypoint_push import WaypointPush_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
