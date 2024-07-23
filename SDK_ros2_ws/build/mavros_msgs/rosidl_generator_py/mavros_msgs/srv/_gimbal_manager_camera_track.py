# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/GimbalManagerCameraTrack.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalManagerCameraTrack_Request(type):
    """Metaclass of message 'GimbalManagerCameraTrack_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CAMERA_TRACK_MODE_POINT': 0,
        'CAMERA_TRACK_MODE_RECTANGLE': 1,
        'CAMERA_TRACK_MODE_STOP_TRACKING': 2,
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
                'mavros_msgs.srv.GimbalManagerCameraTrack_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gimbal_manager_camera_track__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gimbal_manager_camera_track__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gimbal_manager_camera_track__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gimbal_manager_camera_track__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gimbal_manager_camera_track__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CAMERA_TRACK_MODE_POINT': cls.__constants['CAMERA_TRACK_MODE_POINT'],
            'CAMERA_TRACK_MODE_RECTANGLE': cls.__constants['CAMERA_TRACK_MODE_RECTANGLE'],
            'CAMERA_TRACK_MODE_STOP_TRACKING': cls.__constants['CAMERA_TRACK_MODE_STOP_TRACKING'],
        }

    @property
    def CAMERA_TRACK_MODE_POINT(self):
        """Message constant 'CAMERA_TRACK_MODE_POINT'."""
        return Metaclass_GimbalManagerCameraTrack_Request.__constants['CAMERA_TRACK_MODE_POINT']

    @property
    def CAMERA_TRACK_MODE_RECTANGLE(self):
        """Message constant 'CAMERA_TRACK_MODE_RECTANGLE'."""
        return Metaclass_GimbalManagerCameraTrack_Request.__constants['CAMERA_TRACK_MODE_RECTANGLE']

    @property
    def CAMERA_TRACK_MODE_STOP_TRACKING(self):
        """Message constant 'CAMERA_TRACK_MODE_STOP_TRACKING'."""
        return Metaclass_GimbalManagerCameraTrack_Request.__constants['CAMERA_TRACK_MODE_STOP_TRACKING']


class GimbalManagerCameraTrack_Request(metaclass=Metaclass_GimbalManagerCameraTrack_Request):
    """
    Message class 'GimbalManagerCameraTrack_Request'.

    Constants:
      CAMERA_TRACK_MODE_POINT
      CAMERA_TRACK_MODE_RECTANGLE
      CAMERA_TRACK_MODE_STOP_TRACKING
    """

    __slots__ = [
        '_mode',
        '_x',
        '_y',
        '_radius',
        '_top_left_x',
        '_top_left_y',
        '_bottom_right_x',
        '_bottom_right_y',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'x': 'float',
        'y': 'float',
        'radius': 'float',
        'top_left_x': 'float',
        'top_left_y': 'float',
        'bottom_right_x': 'float',
        'bottom_right_y': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.radius = kwargs.get('radius', float())
        self.top_left_x = kwargs.get('top_left_x', float())
        self.top_left_y = kwargs.get('top_left_y', float())
        self.bottom_right_x = kwargs.get('bottom_right_x', float())
        self.bottom_right_y = kwargs.get('bottom_right_y', float())

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
        if self.mode != other.mode:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.radius != other.radius:
            return False
        if self.top_left_x != other.top_left_x:
            return False
        if self.top_left_y != other.top_left_y:
            return False
        if self.bottom_right_x != other.bottom_right_x:
            return False
        if self.bottom_right_y != other.bottom_right_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y = value

    @builtins.property
    def radius(self):
        """Message field 'radius'."""
        return self._radius

    @radius.setter
    def radius(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radius' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'radius' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._radius = value

    @builtins.property
    def top_left_x(self):
        """Message field 'top_left_x'."""
        return self._top_left_x

    @top_left_x.setter
    def top_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'top_left_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'top_left_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._top_left_x = value

    @builtins.property
    def top_left_y(self):
        """Message field 'top_left_y'."""
        return self._top_left_y

    @top_left_y.setter
    def top_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'top_left_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'top_left_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._top_left_y = value

    @builtins.property
    def bottom_right_x(self):
        """Message field 'bottom_right_x'."""
        return self._bottom_right_x

    @bottom_right_x.setter
    def bottom_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bottom_right_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bottom_right_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bottom_right_x = value

    @builtins.property
    def bottom_right_y(self):
        """Message field 'bottom_right_y'."""
        return self._bottom_right_y

    @bottom_right_y.setter
    def bottom_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bottom_right_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bottom_right_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bottom_right_y = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GimbalManagerCameraTrack_Response(type):
    """Metaclass of message 'GimbalManagerCameraTrack_Response'."""

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
                'mavros_msgs.srv.GimbalManagerCameraTrack_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gimbal_manager_camera_track__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gimbal_manager_camera_track__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gimbal_manager_camera_track__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gimbal_manager_camera_track__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gimbal_manager_camera_track__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GimbalManagerCameraTrack_Response(metaclass=Metaclass_GimbalManagerCameraTrack_Response):
    """Message class 'GimbalManagerCameraTrack_Response'."""

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


class Metaclass_GimbalManagerCameraTrack(type):
    """Metaclass of service 'GimbalManagerCameraTrack'."""

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
                'mavros_msgs.srv.GimbalManagerCameraTrack')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__gimbal_manager_camera_track

            from mavros_msgs.srv import _gimbal_manager_camera_track
            if _gimbal_manager_camera_track.Metaclass_GimbalManagerCameraTrack_Request._TYPE_SUPPORT is None:
                _gimbal_manager_camera_track.Metaclass_GimbalManagerCameraTrack_Request.__import_type_support__()
            if _gimbal_manager_camera_track.Metaclass_GimbalManagerCameraTrack_Response._TYPE_SUPPORT is None:
                _gimbal_manager_camera_track.Metaclass_GimbalManagerCameraTrack_Response.__import_type_support__()


class GimbalManagerCameraTrack(metaclass=Metaclass_GimbalManagerCameraTrack):
    from mavros_msgs.srv._gimbal_manager_camera_track import GimbalManagerCameraTrack_Request as Request
    from mavros_msgs.srv._gimbal_manager_camera_track import GimbalManagerCameraTrack_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
