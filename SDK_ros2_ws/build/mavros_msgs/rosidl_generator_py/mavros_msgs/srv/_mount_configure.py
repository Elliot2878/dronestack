# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/MountConfigure.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MountConfigure_Request(type):
    """Metaclass of message 'MountConfigure_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_RETRACT': 0,
        'MODE_NEUTRAL': 1,
        'MODE_MAVLINK_TARGETING': 2,
        'MODE_RC_TARGETING': 3,
        'MODE_GPS_POINT': 4,
        'INPUT_ANGLE_BODY_FRAME': 0,
        'INPUT_ANGULAR_RATE': 1,
        'INPUT_ANGLE_ABSOLUTE_FRAME': 2,
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
                'mavros_msgs.srv.MountConfigure_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__mount_configure__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__mount_configure__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__mount_configure__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__mount_configure__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__mount_configure__request

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_RETRACT': cls.__constants['MODE_RETRACT'],
            'MODE_NEUTRAL': cls.__constants['MODE_NEUTRAL'],
            'MODE_MAVLINK_TARGETING': cls.__constants['MODE_MAVLINK_TARGETING'],
            'MODE_RC_TARGETING': cls.__constants['MODE_RC_TARGETING'],
            'MODE_GPS_POINT': cls.__constants['MODE_GPS_POINT'],
            'INPUT_ANGLE_BODY_FRAME': cls.__constants['INPUT_ANGLE_BODY_FRAME'],
            'INPUT_ANGULAR_RATE': cls.__constants['INPUT_ANGULAR_RATE'],
            'INPUT_ANGLE_ABSOLUTE_FRAME': cls.__constants['INPUT_ANGLE_ABSOLUTE_FRAME'],
        }

    @property
    def MODE_RETRACT(self):
        """Message constant 'MODE_RETRACT'."""
        return Metaclass_MountConfigure_Request.__constants['MODE_RETRACT']

    @property
    def MODE_NEUTRAL(self):
        """Message constant 'MODE_NEUTRAL'."""
        return Metaclass_MountConfigure_Request.__constants['MODE_NEUTRAL']

    @property
    def MODE_MAVLINK_TARGETING(self):
        """Message constant 'MODE_MAVLINK_TARGETING'."""
        return Metaclass_MountConfigure_Request.__constants['MODE_MAVLINK_TARGETING']

    @property
    def MODE_RC_TARGETING(self):
        """Message constant 'MODE_RC_TARGETING'."""
        return Metaclass_MountConfigure_Request.__constants['MODE_RC_TARGETING']

    @property
    def MODE_GPS_POINT(self):
        """Message constant 'MODE_GPS_POINT'."""
        return Metaclass_MountConfigure_Request.__constants['MODE_GPS_POINT']

    @property
    def INPUT_ANGLE_BODY_FRAME(self):
        """Message constant 'INPUT_ANGLE_BODY_FRAME'."""
        return Metaclass_MountConfigure_Request.__constants['INPUT_ANGLE_BODY_FRAME']

    @property
    def INPUT_ANGULAR_RATE(self):
        """Message constant 'INPUT_ANGULAR_RATE'."""
        return Metaclass_MountConfigure_Request.__constants['INPUT_ANGULAR_RATE']

    @property
    def INPUT_ANGLE_ABSOLUTE_FRAME(self):
        """Message constant 'INPUT_ANGLE_ABSOLUTE_FRAME'."""
        return Metaclass_MountConfigure_Request.__constants['INPUT_ANGLE_ABSOLUTE_FRAME']


class MountConfigure_Request(metaclass=Metaclass_MountConfigure_Request):
    """
    Message class 'MountConfigure_Request'.

    Constants:
      MODE_RETRACT
      MODE_NEUTRAL
      MODE_MAVLINK_TARGETING
      MODE_RC_TARGETING
      MODE_GPS_POINT
      INPUT_ANGLE_BODY_FRAME
      INPUT_ANGULAR_RATE
      INPUT_ANGLE_ABSOLUTE_FRAME
    """

    __slots__ = [
        '_header',
        '_mode',
        '_stabilize_roll',
        '_stabilize_pitch',
        '_stabilize_yaw',
        '_roll_input',
        '_pitch_input',
        '_yaw_input',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'mode': 'uint8',
        'stabilize_roll': 'boolean',
        'stabilize_pitch': 'boolean',
        'stabilize_yaw': 'boolean',
        'roll_input': 'uint8',
        'pitch_input': 'uint8',
        'yaw_input': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.mode = kwargs.get('mode', int())
        self.stabilize_roll = kwargs.get('stabilize_roll', bool())
        self.stabilize_pitch = kwargs.get('stabilize_pitch', bool())
        self.stabilize_yaw = kwargs.get('stabilize_yaw', bool())
        self.roll_input = kwargs.get('roll_input', int())
        self.pitch_input = kwargs.get('pitch_input', int())
        self.yaw_input = kwargs.get('yaw_input', int())

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
        if self.mode != other.mode:
            return False
        if self.stabilize_roll != other.stabilize_roll:
            return False
        if self.stabilize_pitch != other.stabilize_pitch:
            return False
        if self.stabilize_yaw != other.stabilize_yaw:
            return False
        if self.roll_input != other.roll_input:
            return False
        if self.pitch_input != other.pitch_input:
            return False
        if self.yaw_input != other.yaw_input:
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

    @builtins.property
    def stabilize_roll(self):
        """Message field 'stabilize_roll'."""
        return self._stabilize_roll

    @stabilize_roll.setter
    def stabilize_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stabilize_roll' field must be of type 'bool'"
        self._stabilize_roll = value

    @builtins.property
    def stabilize_pitch(self):
        """Message field 'stabilize_pitch'."""
        return self._stabilize_pitch

    @stabilize_pitch.setter
    def stabilize_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stabilize_pitch' field must be of type 'bool'"
        self._stabilize_pitch = value

    @builtins.property
    def stabilize_yaw(self):
        """Message field 'stabilize_yaw'."""
        return self._stabilize_yaw

    @stabilize_yaw.setter
    def stabilize_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stabilize_yaw' field must be of type 'bool'"
        self._stabilize_yaw = value

    @builtins.property
    def roll_input(self):
        """Message field 'roll_input'."""
        return self._roll_input

    @roll_input.setter
    def roll_input(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'roll_input' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'roll_input' field must be an unsigned integer in [0, 255]"
        self._roll_input = value

    @builtins.property
    def pitch_input(self):
        """Message field 'pitch_input'."""
        return self._pitch_input

    @pitch_input.setter
    def pitch_input(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pitch_input' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'pitch_input' field must be an unsigned integer in [0, 255]"
        self._pitch_input = value

    @builtins.property
    def yaw_input(self):
        """Message field 'yaw_input'."""
        return self._yaw_input

    @yaw_input.setter
    def yaw_input(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'yaw_input' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'yaw_input' field must be an unsigned integer in [0, 255]"
        self._yaw_input = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MountConfigure_Response(type):
    """Metaclass of message 'MountConfigure_Response'."""

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
                'mavros_msgs.srv.MountConfigure_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__mount_configure__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__mount_configure__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__mount_configure__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__mount_configure__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__mount_configure__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MountConfigure_Response(metaclass=Metaclass_MountConfigure_Response):
    """Message class 'MountConfigure_Response'."""

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


class Metaclass_MountConfigure(type):
    """Metaclass of service 'MountConfigure'."""

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
                'mavros_msgs.srv.MountConfigure')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__mount_configure

            from mavros_msgs.srv import _mount_configure
            if _mount_configure.Metaclass_MountConfigure_Request._TYPE_SUPPORT is None:
                _mount_configure.Metaclass_MountConfigure_Request.__import_type_support__()
            if _mount_configure.Metaclass_MountConfigure_Response._TYPE_SUPPORT is None:
                _mount_configure.Metaclass_MountConfigure_Response.__import_type_support__()


class MountConfigure(metaclass=Metaclass_MountConfigure):
    from mavros_msgs.srv._mount_configure import MountConfigure_Request as Request
    from mavros_msgs.srv._mount_configure import MountConfigure_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
