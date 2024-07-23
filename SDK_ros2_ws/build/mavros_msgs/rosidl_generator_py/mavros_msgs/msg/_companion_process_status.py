# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/CompanionProcessStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CompanionProcessStatus(type):
    """Metaclass of message 'CompanionProcessStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAV_STATE_UNINIT': 0,
        'MAV_STATE_BOOT': 1,
        'MAV_STATE_CALIBRATING': 2,
        'MAV_STATE_STANDBY': 3,
        'MAV_STATE_ACTIVE': 4,
        'MAV_STATE_CRITICAL': 5,
        'MAV_STATE_EMERGENCY': 6,
        'MAV_STATE_POWEROFF': 7,
        'MAV_STATE_FLIGHT_TERMINATION': 8,
        'MAV_COMP_ID_OBSTACLE_AVOIDANCE': 196,
        'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY': 197,
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
                'mavros_msgs.msg.CompanionProcessStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__companion_process_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__companion_process_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__companion_process_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__companion_process_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__companion_process_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAV_STATE_UNINIT': cls.__constants['MAV_STATE_UNINIT'],
            'MAV_STATE_BOOT': cls.__constants['MAV_STATE_BOOT'],
            'MAV_STATE_CALIBRATING': cls.__constants['MAV_STATE_CALIBRATING'],
            'MAV_STATE_STANDBY': cls.__constants['MAV_STATE_STANDBY'],
            'MAV_STATE_ACTIVE': cls.__constants['MAV_STATE_ACTIVE'],
            'MAV_STATE_CRITICAL': cls.__constants['MAV_STATE_CRITICAL'],
            'MAV_STATE_EMERGENCY': cls.__constants['MAV_STATE_EMERGENCY'],
            'MAV_STATE_POWEROFF': cls.__constants['MAV_STATE_POWEROFF'],
            'MAV_STATE_FLIGHT_TERMINATION': cls.__constants['MAV_STATE_FLIGHT_TERMINATION'],
            'MAV_COMP_ID_OBSTACLE_AVOIDANCE': cls.__constants['MAV_COMP_ID_OBSTACLE_AVOIDANCE'],
            'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY': cls.__constants['MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY'],
        }

    @property
    def MAV_STATE_UNINIT(self):
        """Message constant 'MAV_STATE_UNINIT'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_UNINIT']

    @property
    def MAV_STATE_BOOT(self):
        """Message constant 'MAV_STATE_BOOT'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_BOOT']

    @property
    def MAV_STATE_CALIBRATING(self):
        """Message constant 'MAV_STATE_CALIBRATING'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_CALIBRATING']

    @property
    def MAV_STATE_STANDBY(self):
        """Message constant 'MAV_STATE_STANDBY'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_STANDBY']

    @property
    def MAV_STATE_ACTIVE(self):
        """Message constant 'MAV_STATE_ACTIVE'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_ACTIVE']

    @property
    def MAV_STATE_CRITICAL(self):
        """Message constant 'MAV_STATE_CRITICAL'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_CRITICAL']

    @property
    def MAV_STATE_EMERGENCY(self):
        """Message constant 'MAV_STATE_EMERGENCY'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_EMERGENCY']

    @property
    def MAV_STATE_POWEROFF(self):
        """Message constant 'MAV_STATE_POWEROFF'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_POWEROFF']

    @property
    def MAV_STATE_FLIGHT_TERMINATION(self):
        """Message constant 'MAV_STATE_FLIGHT_TERMINATION'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_STATE_FLIGHT_TERMINATION']

    @property
    def MAV_COMP_ID_OBSTACLE_AVOIDANCE(self):
        """Message constant 'MAV_COMP_ID_OBSTACLE_AVOIDANCE'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_COMP_ID_OBSTACLE_AVOIDANCE']

    @property
    def MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY(self):
        """Message constant 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY'."""
        return Metaclass_CompanionProcessStatus.__constants['MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY']


class CompanionProcessStatus(metaclass=Metaclass_CompanionProcessStatus):
    """
    Message class 'CompanionProcessStatus'.

    Constants:
      MAV_STATE_UNINIT
      MAV_STATE_BOOT
      MAV_STATE_CALIBRATING
      MAV_STATE_STANDBY
      MAV_STATE_ACTIVE
      MAV_STATE_CRITICAL
      MAV_STATE_EMERGENCY
      MAV_STATE_POWEROFF
      MAV_STATE_FLIGHT_TERMINATION
      MAV_COMP_ID_OBSTACLE_AVOIDANCE
      MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
    """

    __slots__ = [
        '_header',
        '_state',
        '_component',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'uint8',
        'component': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.state = kwargs.get('state', int())
        self.component = kwargs.get('component', int())

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
        if self.state != other.state:
            return False
        if self.component != other.component:
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

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value

    @builtins.property
    def component(self):
        """Message field 'component'."""
        return self._component

    @component.setter
    def component(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'component' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'component' field must be an unsigned integer in [0, 255]"
        self._component = value
