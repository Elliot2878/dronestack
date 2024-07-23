# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/GimbalManagerSetRoi.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GimbalManagerSetRoi_Request(type):
    """Metaclass of message 'GimbalManagerSetRoi_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ROI_MODE_LOCATION': 0,
        'ROI_MODE_WP_NEXT_OFFSET': 1,
        'ROI_MODE_SYSID': 2,
        'ROI_MODE_NONE': 3,
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
                'mavros_msgs.srv.GimbalManagerSetRoi_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gimbal_manager_set_roi__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gimbal_manager_set_roi__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gimbal_manager_set_roi__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gimbal_manager_set_roi__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gimbal_manager_set_roi__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ROI_MODE_LOCATION': cls.__constants['ROI_MODE_LOCATION'],
            'ROI_MODE_WP_NEXT_OFFSET': cls.__constants['ROI_MODE_WP_NEXT_OFFSET'],
            'ROI_MODE_SYSID': cls.__constants['ROI_MODE_SYSID'],
            'ROI_MODE_NONE': cls.__constants['ROI_MODE_NONE'],
        }

    @property
    def ROI_MODE_LOCATION(self):
        """Message constant 'ROI_MODE_LOCATION'."""
        return Metaclass_GimbalManagerSetRoi_Request.__constants['ROI_MODE_LOCATION']

    @property
    def ROI_MODE_WP_NEXT_OFFSET(self):
        """Message constant 'ROI_MODE_WP_NEXT_OFFSET'."""
        return Metaclass_GimbalManagerSetRoi_Request.__constants['ROI_MODE_WP_NEXT_OFFSET']

    @property
    def ROI_MODE_SYSID(self):
        """Message constant 'ROI_MODE_SYSID'."""
        return Metaclass_GimbalManagerSetRoi_Request.__constants['ROI_MODE_SYSID']

    @property
    def ROI_MODE_NONE(self):
        """Message constant 'ROI_MODE_NONE'."""
        return Metaclass_GimbalManagerSetRoi_Request.__constants['ROI_MODE_NONE']


class GimbalManagerSetRoi_Request(metaclass=Metaclass_GimbalManagerSetRoi_Request):
    """
    Message class 'GimbalManagerSetRoi_Request'.

    Constants:
      ROI_MODE_LOCATION
      ROI_MODE_WP_NEXT_OFFSET
      ROI_MODE_SYSID
      ROI_MODE_NONE
    """

    __slots__ = [
        '_mode',
        '_gimbal_device_id',
        '_latitude',
        '_longitude',
        '_altitude',
        '_pitch_offset',
        '_roll_offset',
        '_yaw_offset',
        '_sysid',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'gimbal_device_id': 'uint8',
        'latitude': 'float',
        'longitude': 'float',
        'altitude': 'float',
        'pitch_offset': 'float',
        'roll_offset': 'float',
        'yaw_offset': 'float',
        'sysid': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        self.gimbal_device_id = kwargs.get('gimbal_device_id', int())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.altitude = kwargs.get('altitude', float())
        self.pitch_offset = kwargs.get('pitch_offset', float())
        self.roll_offset = kwargs.get('roll_offset', float())
        self.yaw_offset = kwargs.get('yaw_offset', float())
        self.sysid = kwargs.get('sysid', int())

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
        if self.gimbal_device_id != other.gimbal_device_id:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.pitch_offset != other.pitch_offset:
            return False
        if self.roll_offset != other.roll_offset:
            return False
        if self.yaw_offset != other.yaw_offset:
            return False
        if self.sysid != other.sysid:
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
    def gimbal_device_id(self):
        """Message field 'gimbal_device_id'."""
        return self._gimbal_device_id

    @gimbal_device_id.setter
    def gimbal_device_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gimbal_device_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gimbal_device_id' field must be an unsigned integer in [0, 255]"
        self._gimbal_device_id = value

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'latitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'longitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._longitude = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'altitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._altitude = value

    @builtins.property
    def pitch_offset(self):
        """Message field 'pitch_offset'."""
        return self._pitch_offset

    @pitch_offset.setter
    def pitch_offset(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_offset' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_offset' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_offset = value

    @builtins.property
    def roll_offset(self):
        """Message field 'roll_offset'."""
        return self._roll_offset

    @roll_offset.setter
    def roll_offset(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_offset' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_offset' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_offset = value

    @builtins.property
    def yaw_offset(self):
        """Message field 'yaw_offset'."""
        return self._yaw_offset

    @yaw_offset.setter
    def yaw_offset(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_offset' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_offset' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_offset = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GimbalManagerSetRoi_Response(type):
    """Metaclass of message 'GimbalManagerSetRoi_Response'."""

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
                'mavros_msgs.srv.GimbalManagerSetRoi_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gimbal_manager_set_roi__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gimbal_manager_set_roi__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gimbal_manager_set_roi__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gimbal_manager_set_roi__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gimbal_manager_set_roi__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GimbalManagerSetRoi_Response(metaclass=Metaclass_GimbalManagerSetRoi_Response):
    """Message class 'GimbalManagerSetRoi_Response'."""

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


class Metaclass_GimbalManagerSetRoi(type):
    """Metaclass of service 'GimbalManagerSetRoi'."""

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
                'mavros_msgs.srv.GimbalManagerSetRoi')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__gimbal_manager_set_roi

            from mavros_msgs.srv import _gimbal_manager_set_roi
            if _gimbal_manager_set_roi.Metaclass_GimbalManagerSetRoi_Request._TYPE_SUPPORT is None:
                _gimbal_manager_set_roi.Metaclass_GimbalManagerSetRoi_Request.__import_type_support__()
            if _gimbal_manager_set_roi.Metaclass_GimbalManagerSetRoi_Response._TYPE_SUPPORT is None:
                _gimbal_manager_set_roi.Metaclass_GimbalManagerSetRoi_Response.__import_type_support__()


class GimbalManagerSetRoi(metaclass=Metaclass_GimbalManagerSetRoi):
    from mavros_msgs.srv._gimbal_manager_set_roi import GimbalManagerSetRoi_Request as Request
    from mavros_msgs.srv._gimbal_manager_set_roi import GimbalManagerSetRoi_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
