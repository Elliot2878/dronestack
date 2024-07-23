# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/SetMavFrame.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetMavFrame_Request(type):
    """Metaclass of message 'SetMavFrame_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FRAME_GLOBAL': 0,
        'FRAME_LOCAL_NED': 1,
        'FRAME_MISSION': 2,
        'FRAME_GLOBAL_RELATIVE_ALT': 3,
        'FRAME_LOCAL_ENU': 4,
        'FRAME_GLOBAL_INT': 5,
        'FRAME_GLOBAL_RELATIVE_ALT_INT': 6,
        'FRAME_LOCAL_OFFSET_NED': 7,
        'FRAME_BODY_NED': 8,
        'FRAME_BODY_OFFSET_NED': 9,
        'FRAME_GLOBAL_TERRAIN_ALT': 10,
        'FRAME_GLOBAL_TERRAIN_ALT_INT': 11,
        'FRAME_BODY_FRD': 12,
        'FRAME_RESERVED_13': 13,
        'FRAME_RESERVED_14': 14,
        'FRAME_RESERVED_15': 15,
        'FRAME_RESERVED_16': 16,
        'FRAME_RESERVED_17': 17,
        'FRAME_RESERVED_18': 18,
        'FRAME_RESERVED_19': 19,
        'FRAME_LOCAL_FRD': 20,
        'FRAME_LOCAL_FLU': 21,
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
                'mavros_msgs.srv.SetMavFrame_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_mav_frame__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_mav_frame__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_mav_frame__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_mav_frame__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_mav_frame__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FRAME_GLOBAL': cls.__constants['FRAME_GLOBAL'],
            'FRAME_LOCAL_NED': cls.__constants['FRAME_LOCAL_NED'],
            'FRAME_MISSION': cls.__constants['FRAME_MISSION'],
            'FRAME_GLOBAL_RELATIVE_ALT': cls.__constants['FRAME_GLOBAL_RELATIVE_ALT'],
            'FRAME_LOCAL_ENU': cls.__constants['FRAME_LOCAL_ENU'],
            'FRAME_GLOBAL_INT': cls.__constants['FRAME_GLOBAL_INT'],
            'FRAME_GLOBAL_RELATIVE_ALT_INT': cls.__constants['FRAME_GLOBAL_RELATIVE_ALT_INT'],
            'FRAME_LOCAL_OFFSET_NED': cls.__constants['FRAME_LOCAL_OFFSET_NED'],
            'FRAME_BODY_NED': cls.__constants['FRAME_BODY_NED'],
            'FRAME_BODY_OFFSET_NED': cls.__constants['FRAME_BODY_OFFSET_NED'],
            'FRAME_GLOBAL_TERRAIN_ALT': cls.__constants['FRAME_GLOBAL_TERRAIN_ALT'],
            'FRAME_GLOBAL_TERRAIN_ALT_INT': cls.__constants['FRAME_GLOBAL_TERRAIN_ALT_INT'],
            'FRAME_BODY_FRD': cls.__constants['FRAME_BODY_FRD'],
            'FRAME_RESERVED_13': cls.__constants['FRAME_RESERVED_13'],
            'FRAME_RESERVED_14': cls.__constants['FRAME_RESERVED_14'],
            'FRAME_RESERVED_15': cls.__constants['FRAME_RESERVED_15'],
            'FRAME_RESERVED_16': cls.__constants['FRAME_RESERVED_16'],
            'FRAME_RESERVED_17': cls.__constants['FRAME_RESERVED_17'],
            'FRAME_RESERVED_18': cls.__constants['FRAME_RESERVED_18'],
            'FRAME_RESERVED_19': cls.__constants['FRAME_RESERVED_19'],
            'FRAME_LOCAL_FRD': cls.__constants['FRAME_LOCAL_FRD'],
            'FRAME_LOCAL_FLU': cls.__constants['FRAME_LOCAL_FLU'],
        }

    @property
    def FRAME_GLOBAL(self):
        """Message constant 'FRAME_GLOBAL'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_GLOBAL']

    @property
    def FRAME_LOCAL_NED(self):
        """Message constant 'FRAME_LOCAL_NED'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_LOCAL_NED']

    @property
    def FRAME_MISSION(self):
        """Message constant 'FRAME_MISSION'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_MISSION']

    @property
    def FRAME_GLOBAL_RELATIVE_ALT(self):
        """Message constant 'FRAME_GLOBAL_RELATIVE_ALT'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_GLOBAL_RELATIVE_ALT']

    @property
    def FRAME_LOCAL_ENU(self):
        """Message constant 'FRAME_LOCAL_ENU'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_LOCAL_ENU']

    @property
    def FRAME_GLOBAL_INT(self):
        """Message constant 'FRAME_GLOBAL_INT'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_GLOBAL_INT']

    @property
    def FRAME_GLOBAL_RELATIVE_ALT_INT(self):
        """Message constant 'FRAME_GLOBAL_RELATIVE_ALT_INT'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_GLOBAL_RELATIVE_ALT_INT']

    @property
    def FRAME_LOCAL_OFFSET_NED(self):
        """Message constant 'FRAME_LOCAL_OFFSET_NED'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_LOCAL_OFFSET_NED']

    @property
    def FRAME_BODY_NED(self):
        """Message constant 'FRAME_BODY_NED'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_BODY_NED']

    @property
    def FRAME_BODY_OFFSET_NED(self):
        """Message constant 'FRAME_BODY_OFFSET_NED'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_BODY_OFFSET_NED']

    @property
    def FRAME_GLOBAL_TERRAIN_ALT(self):
        """Message constant 'FRAME_GLOBAL_TERRAIN_ALT'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_GLOBAL_TERRAIN_ALT']

    @property
    def FRAME_GLOBAL_TERRAIN_ALT_INT(self):
        """Message constant 'FRAME_GLOBAL_TERRAIN_ALT_INT'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_GLOBAL_TERRAIN_ALT_INT']

    @property
    def FRAME_BODY_FRD(self):
        """Message constant 'FRAME_BODY_FRD'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_BODY_FRD']

    @property
    def FRAME_RESERVED_13(self):
        """Message constant 'FRAME_RESERVED_13'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_13']

    @property
    def FRAME_RESERVED_14(self):
        """Message constant 'FRAME_RESERVED_14'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_14']

    @property
    def FRAME_RESERVED_15(self):
        """Message constant 'FRAME_RESERVED_15'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_15']

    @property
    def FRAME_RESERVED_16(self):
        """Message constant 'FRAME_RESERVED_16'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_16']

    @property
    def FRAME_RESERVED_17(self):
        """Message constant 'FRAME_RESERVED_17'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_17']

    @property
    def FRAME_RESERVED_18(self):
        """Message constant 'FRAME_RESERVED_18'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_18']

    @property
    def FRAME_RESERVED_19(self):
        """Message constant 'FRAME_RESERVED_19'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_RESERVED_19']

    @property
    def FRAME_LOCAL_FRD(self):
        """Message constant 'FRAME_LOCAL_FRD'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_LOCAL_FRD']

    @property
    def FRAME_LOCAL_FLU(self):
        """Message constant 'FRAME_LOCAL_FLU'."""
        return Metaclass_SetMavFrame_Request.__constants['FRAME_LOCAL_FLU']


class SetMavFrame_Request(metaclass=Metaclass_SetMavFrame_Request):
    """
    Message class 'SetMavFrame_Request'.

    Constants:
      FRAME_GLOBAL
      FRAME_LOCAL_NED
      FRAME_MISSION
      FRAME_GLOBAL_RELATIVE_ALT
      FRAME_LOCAL_ENU
      FRAME_GLOBAL_INT
      FRAME_GLOBAL_RELATIVE_ALT_INT
      FRAME_LOCAL_OFFSET_NED
      FRAME_BODY_NED
      FRAME_BODY_OFFSET_NED
      FRAME_GLOBAL_TERRAIN_ALT
      FRAME_GLOBAL_TERRAIN_ALT_INT
      FRAME_BODY_FRD
      FRAME_RESERVED_13
      FRAME_RESERVED_14
      FRAME_RESERVED_15
      FRAME_RESERVED_16
      FRAME_RESERVED_17
      FRAME_RESERVED_18
      FRAME_RESERVED_19
      FRAME_LOCAL_FRD
      FRAME_LOCAL_FLU
    """

    __slots__ = [
        '_mav_frame',
    ]

    _fields_and_field_types = {
        'mav_frame': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mav_frame = kwargs.get('mav_frame', int())

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
        if self.mav_frame != other.mav_frame:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mav_frame(self):
        """Message field 'mav_frame'."""
        return self._mav_frame

    @mav_frame.setter
    def mav_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mav_frame' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mav_frame' field must be an unsigned integer in [0, 255]"
        self._mav_frame = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetMavFrame_Response(type):
    """Metaclass of message 'SetMavFrame_Response'."""

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
                'mavros_msgs.srv.SetMavFrame_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_mav_frame__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_mav_frame__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_mav_frame__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_mav_frame__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_mav_frame__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetMavFrame_Response(metaclass=Metaclass_SetMavFrame_Response):
    """Message class 'SetMavFrame_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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


class Metaclass_SetMavFrame(type):
    """Metaclass of service 'SetMavFrame'."""

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
                'mavros_msgs.srv.SetMavFrame')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_mav_frame

            from mavros_msgs.srv import _set_mav_frame
            if _set_mav_frame.Metaclass_SetMavFrame_Request._TYPE_SUPPORT is None:
                _set_mav_frame.Metaclass_SetMavFrame_Request.__import_type_support__()
            if _set_mav_frame.Metaclass_SetMavFrame_Response._TYPE_SUPPORT is None:
                _set_mav_frame.Metaclass_SetMavFrame_Response.__import_type_support__()


class SetMavFrame(metaclass=Metaclass_SetMavFrame):
    from mavros_msgs.srv._set_mav_frame import SetMavFrame_Request as Request
    from mavros_msgs.srv._set_mav_frame import SetMavFrame_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
