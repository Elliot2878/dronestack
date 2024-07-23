# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/ExtendedState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExtendedState(type):
    """Metaclass of message 'ExtendedState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'VTOL_STATE_UNDEFINED': 0,
        'VTOL_STATE_TRANSITION_TO_FW': 1,
        'VTOL_STATE_TRANSITION_TO_MC': 2,
        'VTOL_STATE_MC': 3,
        'VTOL_STATE_FW': 4,
        'LANDED_STATE_UNDEFINED': 0,
        'LANDED_STATE_ON_GROUND': 1,
        'LANDED_STATE_IN_AIR': 2,
        'LANDED_STATE_TAKEOFF': 3,
        'LANDED_STATE_LANDING': 4,
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
                'mavros_msgs.msg.ExtendedState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__extended_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__extended_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__extended_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__extended_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__extended_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'VTOL_STATE_UNDEFINED': cls.__constants['VTOL_STATE_UNDEFINED'],
            'VTOL_STATE_TRANSITION_TO_FW': cls.__constants['VTOL_STATE_TRANSITION_TO_FW'],
            'VTOL_STATE_TRANSITION_TO_MC': cls.__constants['VTOL_STATE_TRANSITION_TO_MC'],
            'VTOL_STATE_MC': cls.__constants['VTOL_STATE_MC'],
            'VTOL_STATE_FW': cls.__constants['VTOL_STATE_FW'],
            'LANDED_STATE_UNDEFINED': cls.__constants['LANDED_STATE_UNDEFINED'],
            'LANDED_STATE_ON_GROUND': cls.__constants['LANDED_STATE_ON_GROUND'],
            'LANDED_STATE_IN_AIR': cls.__constants['LANDED_STATE_IN_AIR'],
            'LANDED_STATE_TAKEOFF': cls.__constants['LANDED_STATE_TAKEOFF'],
            'LANDED_STATE_LANDING': cls.__constants['LANDED_STATE_LANDING'],
        }

    @property
    def VTOL_STATE_UNDEFINED(self):
        """Message constant 'VTOL_STATE_UNDEFINED'."""
        return Metaclass_ExtendedState.__constants['VTOL_STATE_UNDEFINED']

    @property
    def VTOL_STATE_TRANSITION_TO_FW(self):
        """Message constant 'VTOL_STATE_TRANSITION_TO_FW'."""
        return Metaclass_ExtendedState.__constants['VTOL_STATE_TRANSITION_TO_FW']

    @property
    def VTOL_STATE_TRANSITION_TO_MC(self):
        """Message constant 'VTOL_STATE_TRANSITION_TO_MC'."""
        return Metaclass_ExtendedState.__constants['VTOL_STATE_TRANSITION_TO_MC']

    @property
    def VTOL_STATE_MC(self):
        """Message constant 'VTOL_STATE_MC'."""
        return Metaclass_ExtendedState.__constants['VTOL_STATE_MC']

    @property
    def VTOL_STATE_FW(self):
        """Message constant 'VTOL_STATE_FW'."""
        return Metaclass_ExtendedState.__constants['VTOL_STATE_FW']

    @property
    def LANDED_STATE_UNDEFINED(self):
        """Message constant 'LANDED_STATE_UNDEFINED'."""
        return Metaclass_ExtendedState.__constants['LANDED_STATE_UNDEFINED']

    @property
    def LANDED_STATE_ON_GROUND(self):
        """Message constant 'LANDED_STATE_ON_GROUND'."""
        return Metaclass_ExtendedState.__constants['LANDED_STATE_ON_GROUND']

    @property
    def LANDED_STATE_IN_AIR(self):
        """Message constant 'LANDED_STATE_IN_AIR'."""
        return Metaclass_ExtendedState.__constants['LANDED_STATE_IN_AIR']

    @property
    def LANDED_STATE_TAKEOFF(self):
        """Message constant 'LANDED_STATE_TAKEOFF'."""
        return Metaclass_ExtendedState.__constants['LANDED_STATE_TAKEOFF']

    @property
    def LANDED_STATE_LANDING(self):
        """Message constant 'LANDED_STATE_LANDING'."""
        return Metaclass_ExtendedState.__constants['LANDED_STATE_LANDING']


class ExtendedState(metaclass=Metaclass_ExtendedState):
    """
    Message class 'ExtendedState'.

    Constants:
      VTOL_STATE_UNDEFINED
      VTOL_STATE_TRANSITION_TO_FW
      VTOL_STATE_TRANSITION_TO_MC
      VTOL_STATE_MC
      VTOL_STATE_FW
      LANDED_STATE_UNDEFINED
      LANDED_STATE_ON_GROUND
      LANDED_STATE_IN_AIR
      LANDED_STATE_TAKEOFF
      LANDED_STATE_LANDING
    """

    __slots__ = [
        '_header',
        '_vtol_state',
        '_landed_state',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'vtol_state': 'uint8',
        'landed_state': 'uint8',
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
        self.vtol_state = kwargs.get('vtol_state', int())
        self.landed_state = kwargs.get('landed_state', int())

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
        if self.vtol_state != other.vtol_state:
            return False
        if self.landed_state != other.landed_state:
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
    def vtol_state(self):
        """Message field 'vtol_state'."""
        return self._vtol_state

    @vtol_state.setter
    def vtol_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vtol_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'vtol_state' field must be an unsigned integer in [0, 255]"
        self._vtol_state = value

    @builtins.property
    def landed_state(self):
        """Message field 'landed_state'."""
        return self._landed_state

    @landed_state.setter
    def landed_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'landed_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'landed_state' field must be an unsigned integer in [0, 255]"
        self._landed_state = value
