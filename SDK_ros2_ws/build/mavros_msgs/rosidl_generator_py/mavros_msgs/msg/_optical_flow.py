# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/OpticalFlow.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OpticalFlow(type):
    """Metaclass of message 'OpticalFlow'."""

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
                'mavros_msgs.msg.OpticalFlow')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__optical_flow
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__optical_flow
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__optical_flow
            cls._TYPE_SUPPORT = module.type_support_msg__msg__optical_flow
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__optical_flow

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

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


class OpticalFlow(metaclass=Metaclass_OpticalFlow):
    """Message class 'OpticalFlow'."""

    __slots__ = [
        '_header',
        '_flow',
        '_flow_comp_m',
        '_quality',
        '_ground_distance',
        '_flow_rate',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'flow': 'geometry_msgs/Vector3',
        'flow_comp_m': 'geometry_msgs/Vector3',
        'quality': 'uint8',
        'ground_distance': 'float',
        'flow_rate': 'geometry_msgs/Vector3',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Vector3
        self.flow = kwargs.get('flow', Vector3())
        from geometry_msgs.msg import Vector3
        self.flow_comp_m = kwargs.get('flow_comp_m', Vector3())
        self.quality = kwargs.get('quality', int())
        self.ground_distance = kwargs.get('ground_distance', float())
        from geometry_msgs.msg import Vector3
        self.flow_rate = kwargs.get('flow_rate', Vector3())

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
        if self.flow != other.flow:
            return False
        if self.flow_comp_m != other.flow_comp_m:
            return False
        if self.quality != other.quality:
            return False
        if self.ground_distance != other.ground_distance:
            return False
        if self.flow_rate != other.flow_rate:
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
    def flow(self):
        """Message field 'flow'."""
        return self._flow

    @flow.setter
    def flow(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'flow' field must be a sub message of type 'Vector3'"
        self._flow = value

    @builtins.property
    def flow_comp_m(self):
        """Message field 'flow_comp_m'."""
        return self._flow_comp_m

    @flow_comp_m.setter
    def flow_comp_m(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'flow_comp_m' field must be a sub message of type 'Vector3'"
        self._flow_comp_m = value

    @builtins.property
    def quality(self):
        """Message field 'quality'."""
        return self._quality

    @quality.setter
    def quality(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quality' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'quality' field must be an unsigned integer in [0, 255]"
        self._quality = value

    @builtins.property
    def ground_distance(self):
        """Message field 'ground_distance'."""
        return self._ground_distance

    @ground_distance.setter
    def ground_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ground_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ground_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ground_distance = value

    @builtins.property
    def flow_rate(self):
        """Message field 'flow_rate'."""
        return self._flow_rate

    @flow_rate.setter
    def flow_rate(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'flow_rate' field must be a sub message of type 'Vector3'"
        self._flow_rate = value
