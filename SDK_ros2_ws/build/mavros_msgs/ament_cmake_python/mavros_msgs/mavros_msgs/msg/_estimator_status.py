# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/EstimatorStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EstimatorStatus(type):
    """Metaclass of message 'EstimatorStatus'."""

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
                'mavros_msgs.msg.EstimatorStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__estimator_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__estimator_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__estimator_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__estimator_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__estimator_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EstimatorStatus(metaclass=Metaclass_EstimatorStatus):
    """Message class 'EstimatorStatus'."""

    __slots__ = [
        '_header',
        '_attitude_status_flag',
        '_velocity_horiz_status_flag',
        '_velocity_vert_status_flag',
        '_pos_horiz_rel_status_flag',
        '_pos_horiz_abs_status_flag',
        '_pos_vert_abs_status_flag',
        '_pos_vert_agl_status_flag',
        '_const_pos_mode_status_flag',
        '_pred_pos_horiz_rel_status_flag',
        '_pred_pos_horiz_abs_status_flag',
        '_gps_glitch_status_flag',
        '_accel_error_status_flag',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'attitude_status_flag': 'boolean',
        'velocity_horiz_status_flag': 'boolean',
        'velocity_vert_status_flag': 'boolean',
        'pos_horiz_rel_status_flag': 'boolean',
        'pos_horiz_abs_status_flag': 'boolean',
        'pos_vert_abs_status_flag': 'boolean',
        'pos_vert_agl_status_flag': 'boolean',
        'const_pos_mode_status_flag': 'boolean',
        'pred_pos_horiz_rel_status_flag': 'boolean',
        'pred_pos_horiz_abs_status_flag': 'boolean',
        'gps_glitch_status_flag': 'boolean',
        'accel_error_status_flag': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.attitude_status_flag = kwargs.get('attitude_status_flag', bool())
        self.velocity_horiz_status_flag = kwargs.get('velocity_horiz_status_flag', bool())
        self.velocity_vert_status_flag = kwargs.get('velocity_vert_status_flag', bool())
        self.pos_horiz_rel_status_flag = kwargs.get('pos_horiz_rel_status_flag', bool())
        self.pos_horiz_abs_status_flag = kwargs.get('pos_horiz_abs_status_flag', bool())
        self.pos_vert_abs_status_flag = kwargs.get('pos_vert_abs_status_flag', bool())
        self.pos_vert_agl_status_flag = kwargs.get('pos_vert_agl_status_flag', bool())
        self.const_pos_mode_status_flag = kwargs.get('const_pos_mode_status_flag', bool())
        self.pred_pos_horiz_rel_status_flag = kwargs.get('pred_pos_horiz_rel_status_flag', bool())
        self.pred_pos_horiz_abs_status_flag = kwargs.get('pred_pos_horiz_abs_status_flag', bool())
        self.gps_glitch_status_flag = kwargs.get('gps_glitch_status_flag', bool())
        self.accel_error_status_flag = kwargs.get('accel_error_status_flag', bool())

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
        if self.attitude_status_flag != other.attitude_status_flag:
            return False
        if self.velocity_horiz_status_flag != other.velocity_horiz_status_flag:
            return False
        if self.velocity_vert_status_flag != other.velocity_vert_status_flag:
            return False
        if self.pos_horiz_rel_status_flag != other.pos_horiz_rel_status_flag:
            return False
        if self.pos_horiz_abs_status_flag != other.pos_horiz_abs_status_flag:
            return False
        if self.pos_vert_abs_status_flag != other.pos_vert_abs_status_flag:
            return False
        if self.pos_vert_agl_status_flag != other.pos_vert_agl_status_flag:
            return False
        if self.const_pos_mode_status_flag != other.const_pos_mode_status_flag:
            return False
        if self.pred_pos_horiz_rel_status_flag != other.pred_pos_horiz_rel_status_flag:
            return False
        if self.pred_pos_horiz_abs_status_flag != other.pred_pos_horiz_abs_status_flag:
            return False
        if self.gps_glitch_status_flag != other.gps_glitch_status_flag:
            return False
        if self.accel_error_status_flag != other.accel_error_status_flag:
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
    def attitude_status_flag(self):
        """Message field 'attitude_status_flag'."""
        return self._attitude_status_flag

    @attitude_status_flag.setter
    def attitude_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'attitude_status_flag' field must be of type 'bool'"
        self._attitude_status_flag = value

    @builtins.property
    def velocity_horiz_status_flag(self):
        """Message field 'velocity_horiz_status_flag'."""
        return self._velocity_horiz_status_flag

    @velocity_horiz_status_flag.setter
    def velocity_horiz_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'velocity_horiz_status_flag' field must be of type 'bool'"
        self._velocity_horiz_status_flag = value

    @builtins.property
    def velocity_vert_status_flag(self):
        """Message field 'velocity_vert_status_flag'."""
        return self._velocity_vert_status_flag

    @velocity_vert_status_flag.setter
    def velocity_vert_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'velocity_vert_status_flag' field must be of type 'bool'"
        self._velocity_vert_status_flag = value

    @builtins.property
    def pos_horiz_rel_status_flag(self):
        """Message field 'pos_horiz_rel_status_flag'."""
        return self._pos_horiz_rel_status_flag

    @pos_horiz_rel_status_flag.setter
    def pos_horiz_rel_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pos_horiz_rel_status_flag' field must be of type 'bool'"
        self._pos_horiz_rel_status_flag = value

    @builtins.property
    def pos_horiz_abs_status_flag(self):
        """Message field 'pos_horiz_abs_status_flag'."""
        return self._pos_horiz_abs_status_flag

    @pos_horiz_abs_status_flag.setter
    def pos_horiz_abs_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pos_horiz_abs_status_flag' field must be of type 'bool'"
        self._pos_horiz_abs_status_flag = value

    @builtins.property
    def pos_vert_abs_status_flag(self):
        """Message field 'pos_vert_abs_status_flag'."""
        return self._pos_vert_abs_status_flag

    @pos_vert_abs_status_flag.setter
    def pos_vert_abs_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pos_vert_abs_status_flag' field must be of type 'bool'"
        self._pos_vert_abs_status_flag = value

    @builtins.property
    def pos_vert_agl_status_flag(self):
        """Message field 'pos_vert_agl_status_flag'."""
        return self._pos_vert_agl_status_flag

    @pos_vert_agl_status_flag.setter
    def pos_vert_agl_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pos_vert_agl_status_flag' field must be of type 'bool'"
        self._pos_vert_agl_status_flag = value

    @builtins.property
    def const_pos_mode_status_flag(self):
        """Message field 'const_pos_mode_status_flag'."""
        return self._const_pos_mode_status_flag

    @const_pos_mode_status_flag.setter
    def const_pos_mode_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'const_pos_mode_status_flag' field must be of type 'bool'"
        self._const_pos_mode_status_flag = value

    @builtins.property
    def pred_pos_horiz_rel_status_flag(self):
        """Message field 'pred_pos_horiz_rel_status_flag'."""
        return self._pred_pos_horiz_rel_status_flag

    @pred_pos_horiz_rel_status_flag.setter
    def pred_pos_horiz_rel_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pred_pos_horiz_rel_status_flag' field must be of type 'bool'"
        self._pred_pos_horiz_rel_status_flag = value

    @builtins.property
    def pred_pos_horiz_abs_status_flag(self):
        """Message field 'pred_pos_horiz_abs_status_flag'."""
        return self._pred_pos_horiz_abs_status_flag

    @pred_pos_horiz_abs_status_flag.setter
    def pred_pos_horiz_abs_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pred_pos_horiz_abs_status_flag' field must be of type 'bool'"
        self._pred_pos_horiz_abs_status_flag = value

    @builtins.property
    def gps_glitch_status_flag(self):
        """Message field 'gps_glitch_status_flag'."""
        return self._gps_glitch_status_flag

    @gps_glitch_status_flag.setter
    def gps_glitch_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gps_glitch_status_flag' field must be of type 'bool'"
        self._gps_glitch_status_flag = value

    @builtins.property
    def accel_error_status_flag(self):
        """Message field 'accel_error_status_flag'."""
        return self._accel_error_status_flag

    @accel_error_status_flag.setter
    def accel_error_status_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accel_error_status_flag' field must be of type 'bool'"
        self._accel_error_status_flag = value
