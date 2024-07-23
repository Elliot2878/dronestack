# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/LandingTarget.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'angle'
# Member 'size'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LandingTarget(type):
    """Metaclass of message 'LandingTarget'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GLOBAL': 0,
        'LOCAL_NED': 2,
        'MISSION': 3,
        'GLOBAL_RELATIVE_ALT': 4,
        'LOCAL_ENU': 5,
        'GLOBAL_INT': 6,
        'GLOBAL_RELATIVE_ALT_INT': 7,
        'LOCAL_OFFSET_NED': 8,
        'BODY_NED': 9,
        'BODY_OFFSET_NED': 10,
        'GLOBAL_TERRAIN_ALT': 11,
        'GLOBAL_TERRAIN_ALT_INT': 12,
        'LIGHT_BEACON': 0,
        'RADIO_BEACON': 1,
        'VISION_FIDUCIAL': 2,
        'VISION_OTHER': 3,
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
                'mavros_msgs.msg.LandingTarget')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__landing_target
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__landing_target
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__landing_target
            cls._TYPE_SUPPORT = module.type_support_msg__msg__landing_target
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__landing_target

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GLOBAL': cls.__constants['GLOBAL'],
            'LOCAL_NED': cls.__constants['LOCAL_NED'],
            'MISSION': cls.__constants['MISSION'],
            'GLOBAL_RELATIVE_ALT': cls.__constants['GLOBAL_RELATIVE_ALT'],
            'LOCAL_ENU': cls.__constants['LOCAL_ENU'],
            'GLOBAL_INT': cls.__constants['GLOBAL_INT'],
            'GLOBAL_RELATIVE_ALT_INT': cls.__constants['GLOBAL_RELATIVE_ALT_INT'],
            'LOCAL_OFFSET_NED': cls.__constants['LOCAL_OFFSET_NED'],
            'BODY_NED': cls.__constants['BODY_NED'],
            'BODY_OFFSET_NED': cls.__constants['BODY_OFFSET_NED'],
            'GLOBAL_TERRAIN_ALT': cls.__constants['GLOBAL_TERRAIN_ALT'],
            'GLOBAL_TERRAIN_ALT_INT': cls.__constants['GLOBAL_TERRAIN_ALT_INT'],
            'LIGHT_BEACON': cls.__constants['LIGHT_BEACON'],
            'RADIO_BEACON': cls.__constants['RADIO_BEACON'],
            'VISION_FIDUCIAL': cls.__constants['VISION_FIDUCIAL'],
            'VISION_OTHER': cls.__constants['VISION_OTHER'],
        }

    @property
    def GLOBAL(self):
        """Message constant 'GLOBAL'."""
        return Metaclass_LandingTarget.__constants['GLOBAL']

    @property
    def LOCAL_NED(self):
        """Message constant 'LOCAL_NED'."""
        return Metaclass_LandingTarget.__constants['LOCAL_NED']

    @property
    def MISSION(self):
        """Message constant 'MISSION'."""
        return Metaclass_LandingTarget.__constants['MISSION']

    @property
    def GLOBAL_RELATIVE_ALT(self):
        """Message constant 'GLOBAL_RELATIVE_ALT'."""
        return Metaclass_LandingTarget.__constants['GLOBAL_RELATIVE_ALT']

    @property
    def LOCAL_ENU(self):
        """Message constant 'LOCAL_ENU'."""
        return Metaclass_LandingTarget.__constants['LOCAL_ENU']

    @property
    def GLOBAL_INT(self):
        """Message constant 'GLOBAL_INT'."""
        return Metaclass_LandingTarget.__constants['GLOBAL_INT']

    @property
    def GLOBAL_RELATIVE_ALT_INT(self):
        """Message constant 'GLOBAL_RELATIVE_ALT_INT'."""
        return Metaclass_LandingTarget.__constants['GLOBAL_RELATIVE_ALT_INT']

    @property
    def LOCAL_OFFSET_NED(self):
        """Message constant 'LOCAL_OFFSET_NED'."""
        return Metaclass_LandingTarget.__constants['LOCAL_OFFSET_NED']

    @property
    def BODY_NED(self):
        """Message constant 'BODY_NED'."""
        return Metaclass_LandingTarget.__constants['BODY_NED']

    @property
    def BODY_OFFSET_NED(self):
        """Message constant 'BODY_OFFSET_NED'."""
        return Metaclass_LandingTarget.__constants['BODY_OFFSET_NED']

    @property
    def GLOBAL_TERRAIN_ALT(self):
        """Message constant 'GLOBAL_TERRAIN_ALT'."""
        return Metaclass_LandingTarget.__constants['GLOBAL_TERRAIN_ALT']

    @property
    def GLOBAL_TERRAIN_ALT_INT(self):
        """Message constant 'GLOBAL_TERRAIN_ALT_INT'."""
        return Metaclass_LandingTarget.__constants['GLOBAL_TERRAIN_ALT_INT']

    @property
    def LIGHT_BEACON(self):
        """Message constant 'LIGHT_BEACON'."""
        return Metaclass_LandingTarget.__constants['LIGHT_BEACON']

    @property
    def RADIO_BEACON(self):
        """Message constant 'RADIO_BEACON'."""
        return Metaclass_LandingTarget.__constants['RADIO_BEACON']

    @property
    def VISION_FIDUCIAL(self):
        """Message constant 'VISION_FIDUCIAL'."""
        return Metaclass_LandingTarget.__constants['VISION_FIDUCIAL']

    @property
    def VISION_OTHER(self):
        """Message constant 'VISION_OTHER'."""
        return Metaclass_LandingTarget.__constants['VISION_OTHER']


class LandingTarget(metaclass=Metaclass_LandingTarget):
    """
    Message class 'LandingTarget'.

    Constants:
      GLOBAL
      LOCAL_NED
      MISSION
      GLOBAL_RELATIVE_ALT
      LOCAL_ENU
      GLOBAL_INT
      GLOBAL_RELATIVE_ALT_INT
      LOCAL_OFFSET_NED
      BODY_NED
      BODY_OFFSET_NED
      GLOBAL_TERRAIN_ALT
      GLOBAL_TERRAIN_ALT_INT
      LIGHT_BEACON
      RADIO_BEACON
      VISION_FIDUCIAL
      VISION_OTHER
    """

    __slots__ = [
        '_header',
        '_target_num',
        '_frame',
        '_angle',
        '_distance',
        '_size',
        '_pose',
        '_type',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'target_num': 'uint8',
        'frame': 'uint8',
        'angle': 'float[2]',
        'distance': 'float',
        'size': 'float[2]',
        'pose': 'geometry_msgs/Pose',
        'type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.target_num = kwargs.get('target_num', int())
        self.frame = kwargs.get('frame', int())
        if 'angle' not in kwargs:
            self.angle = numpy.zeros(2, dtype=numpy.float32)
        else:
            self.angle = numpy.array(kwargs.get('angle'), dtype=numpy.float32)
            assert self.angle.shape == (2, )
        self.distance = kwargs.get('distance', float())
        if 'size' not in kwargs:
            self.size = numpy.zeros(2, dtype=numpy.float32)
        else:
            self.size = numpy.array(kwargs.get('size'), dtype=numpy.float32)
            assert self.size.shape == (2, )
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        self.type = kwargs.get('type', int())

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
        if self.target_num != other.target_num:
            return False
        if self.frame != other.frame:
            return False
        if all(self.angle != other.angle):
            return False
        if self.distance != other.distance:
            return False
        if all(self.size != other.size):
            return False
        if self.pose != other.pose:
            return False
        if self.type != other.type:
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
    def target_num(self):
        """Message field 'target_num'."""
        return self._target_num

    @target_num.setter
    def target_num(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_num' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'target_num' field must be an unsigned integer in [0, 255]"
        self._target_num = value

    @builtins.property
    def frame(self):
        """Message field 'frame'."""
        return self._frame

    @frame.setter
    def frame(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frame' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'frame' field must be an unsigned integer in [0, 255]"
        self._frame = value

    @builtins.property
    def angle(self):
        """Message field 'angle'."""
        return self._angle

    @angle.setter
    def angle(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'angle' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'angle' numpy.ndarray() must have a size of 2"
            self._angle = value
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
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'angle' field must be a set or sequence with length 2 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._angle = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance = value

    @builtins.property
    def size(self):
        """Message field 'size'."""
        return self._size

    @size.setter
    def size(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'size' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'size' numpy.ndarray() must have a size of 2"
            self._size = value
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
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'size' field must be a set or sequence with length 2 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._size = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value

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
