# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/PlayTuneV2.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlayTuneV2(type):
    """Metaclass of message 'PlayTuneV2'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'QBASIC1_1': 1,
        'MML_MODERN': 2,
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
                'mavros_msgs.msg.PlayTuneV2')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__play_tune_v2
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__play_tune_v2
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__play_tune_v2
            cls._TYPE_SUPPORT = module.type_support_msg__msg__play_tune_v2
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__play_tune_v2

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'QBASIC1_1': cls.__constants['QBASIC1_1'],
            'MML_MODERN': cls.__constants['MML_MODERN'],
        }

    @property
    def QBASIC1_1(self):
        """Message constant 'QBASIC1_1'."""
        return Metaclass_PlayTuneV2.__constants['QBASIC1_1']

    @property
    def MML_MODERN(self):
        """Message constant 'MML_MODERN'."""
        return Metaclass_PlayTuneV2.__constants['MML_MODERN']


class PlayTuneV2(metaclass=Metaclass_PlayTuneV2):
    """
    Message class 'PlayTuneV2'.

    Constants:
      QBASIC1_1
      MML_MODERN
    """

    __slots__ = [
        '_format',
        '_tune',
    ]

    _fields_and_field_types = {
        'format': 'uint8',
        'tune': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.format = kwargs.get('format', int())
        self.tune = kwargs.get('tune', str())

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
        if self.format != other.format:
            return False
        if self.tune != other.tune:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def format(self):  # noqa: A003
        """Message field 'format'."""
        return self._format

    @format.setter  # noqa: A003
    def format(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'format' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'format' field must be an unsigned integer in [0, 255]"
        self._format = value

    @builtins.property
    def tune(self):
        """Message field 'tune'."""
        return self._tune

    @tune.setter
    def tune(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'tune' field must be of type 'str'"
        self._tune = value
