# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/Trajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'point_valid'
# Member 'command'
# Member 'time_horizon'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Trajectory(type):
    """Metaclass of message 'Trajectory'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS': 0,
        'MAV_TRAJECTORY_REPRESENTATION_BEZIER': 1,
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
                'mavros_msgs.msg.Trajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trajectory
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trajectory
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trajectory
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trajectory
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trajectory

            from mavros_msgs.msg import PositionTarget
            if PositionTarget.__class__._TYPE_SUPPORT is None:
                PositionTarget.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS': cls.__constants['MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS'],
            'MAV_TRAJECTORY_REPRESENTATION_BEZIER': cls.__constants['MAV_TRAJECTORY_REPRESENTATION_BEZIER'],
        }

    @property
    def MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS(self):
        """Message constant 'MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS'."""
        return Metaclass_Trajectory.__constants['MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS']

    @property
    def MAV_TRAJECTORY_REPRESENTATION_BEZIER(self):
        """Message constant 'MAV_TRAJECTORY_REPRESENTATION_BEZIER'."""
        return Metaclass_Trajectory.__constants['MAV_TRAJECTORY_REPRESENTATION_BEZIER']


class Trajectory(metaclass=Metaclass_Trajectory):
    """
    Message class 'Trajectory'.

    Constants:
      MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS
      MAV_TRAJECTORY_REPRESENTATION_BEZIER
    """

    __slots__ = [
        '_header',
        '_type',
        '_point_1',
        '_point_2',
        '_point_3',
        '_point_4',
        '_point_5',
        '_point_valid',
        '_command',
        '_time_horizon',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'type': 'uint8',
        'point_1': 'mavros_msgs/PositionTarget',
        'point_2': 'mavros_msgs/PositionTarget',
        'point_3': 'mavros_msgs/PositionTarget',
        'point_4': 'mavros_msgs/PositionTarget',
        'point_5': 'mavros_msgs/PositionTarget',
        'point_valid': 'uint8[5]',
        'command': 'uint16[5]',
        'time_horizon': 'float[5]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'PositionTarget'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'PositionTarget'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'PositionTarget'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'PositionTarget'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mavros_msgs', 'msg'], 'PositionTarget'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint16'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 5),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.type = kwargs.get('type', int())
        from mavros_msgs.msg import PositionTarget
        self.point_1 = kwargs.get('point_1', PositionTarget())
        from mavros_msgs.msg import PositionTarget
        self.point_2 = kwargs.get('point_2', PositionTarget())
        from mavros_msgs.msg import PositionTarget
        self.point_3 = kwargs.get('point_3', PositionTarget())
        from mavros_msgs.msg import PositionTarget
        self.point_4 = kwargs.get('point_4', PositionTarget())
        from mavros_msgs.msg import PositionTarget
        self.point_5 = kwargs.get('point_5', PositionTarget())
        if 'point_valid' not in kwargs:
            self.point_valid = numpy.zeros(5, dtype=numpy.uint8)
        else:
            self.point_valid = numpy.array(kwargs.get('point_valid'), dtype=numpy.uint8)
            assert self.point_valid.shape == (5, )
        if 'command' not in kwargs:
            self.command = numpy.zeros(5, dtype=numpy.uint16)
        else:
            self.command = numpy.array(kwargs.get('command'), dtype=numpy.uint16)
            assert self.command.shape == (5, )
        if 'time_horizon' not in kwargs:
            self.time_horizon = numpy.zeros(5, dtype=numpy.float32)
        else:
            self.time_horizon = numpy.array(kwargs.get('time_horizon'), dtype=numpy.float32)
            assert self.time_horizon.shape == (5, )

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
        if self.header != other.header:
            return False
        if self.type != other.type:
            return False
        if self.point_1 != other.point_1:
            return False
        if self.point_2 != other.point_2:
            return False
        if self.point_3 != other.point_3:
            return False
        if self.point_4 != other.point_4:
            return False
        if self.point_5 != other.point_5:
            return False
        if all(self.point_valid != other.point_valid):
            return False
        if all(self.command != other.command):
            return False
        if all(self.time_horizon != other.time_horizon):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value

    @builtins.property
    def point_1(self):
        """Message field 'point_1'."""
        return self._point_1

    @point_1.setter
    def point_1(self, value):
        if __debug__:
            from mavros_msgs.msg import PositionTarget
            assert \
                isinstance(value, PositionTarget), \
                "The 'point_1' field must be a sub message of type 'PositionTarget'"
        self._point_1 = value

    @builtins.property
    def point_2(self):
        """Message field 'point_2'."""
        return self._point_2

    @point_2.setter
    def point_2(self, value):
        if __debug__:
            from mavros_msgs.msg import PositionTarget
            assert \
                isinstance(value, PositionTarget), \
                "The 'point_2' field must be a sub message of type 'PositionTarget'"
        self._point_2 = value

    @builtins.property
    def point_3(self):
        """Message field 'point_3'."""
        return self._point_3

    @point_3.setter
    def point_3(self, value):
        if __debug__:
            from mavros_msgs.msg import PositionTarget
            assert \
                isinstance(value, PositionTarget), \
                "The 'point_3' field must be a sub message of type 'PositionTarget'"
        self._point_3 = value

    @builtins.property
    def point_4(self):
        """Message field 'point_4'."""
        return self._point_4

    @point_4.setter
    def point_4(self, value):
        if __debug__:
            from mavros_msgs.msg import PositionTarget
            assert \
                isinstance(value, PositionTarget), \
                "The 'point_4' field must be a sub message of type 'PositionTarget'"
        self._point_4 = value

    @builtins.property
    def point_5(self):
        """Message field 'point_5'."""
        return self._point_5

    @point_5.setter
    def point_5(self, value):
        if __debug__:
            from mavros_msgs.msg import PositionTarget
            assert \
                isinstance(value, PositionTarget), \
                "The 'point_5' field must be a sub message of type 'PositionTarget'"
        self._point_5 = value

    @builtins.property
    def point_valid(self):
        """Message field 'point_valid'."""
        return self._point_valid

    @point_valid.setter
    def point_valid(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'point_valid' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 5, \
                "The 'point_valid' numpy.ndarray() must have a size of 5"
            self._point_valid = value
            return
        if __debug__:
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'point_valid' field must be a set or sequence with length 5 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._point_valid = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint16, \
                "The 'command' numpy.ndarray() must have the dtype of 'numpy.uint16'"
            assert value.size == 5, \
                "The 'command' numpy.ndarray() must have a size of 5"
            self._command = value
            return
        if __debug__:
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'command' field must be a set or sequence with length 5 and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._command = numpy.array(value, dtype=numpy.uint16)

    @builtins.property
    def time_horizon(self):
        """Message field 'time_horizon'."""
        return self._time_horizon

    @time_horizon.setter
    def time_horizon(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'time_horizon' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 5, \
                "The 'time_horizon' numpy.ndarray() must have a size of 5"
            self._time_horizon = value
            return
        if __debug__:
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
                 len(value) == 5 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'time_horizon' field must be a set or sequence with length 5 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._time_horizon = numpy.array(value, dtype=numpy.float32)
