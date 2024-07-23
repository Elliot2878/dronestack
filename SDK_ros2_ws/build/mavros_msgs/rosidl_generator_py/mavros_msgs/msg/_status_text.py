# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/StatusText.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StatusText(type):
    """Metaclass of message 'StatusText'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'EMERGENCY': 0,
        'ALERT': 1,
        'CRITICAL': 2,
        'ERROR': 3,
        'WARNING': 4,
        'NOTICE': 5,
        'INFO': 6,
        'DEBUG': 7,
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
                'mavros_msgs.msg.StatusText')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status_text
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status_text
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status_text
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status_text
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status_text

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'EMERGENCY': cls.__constants['EMERGENCY'],
            'ALERT': cls.__constants['ALERT'],
            'CRITICAL': cls.__constants['CRITICAL'],
            'ERROR': cls.__constants['ERROR'],
            'WARNING': cls.__constants['WARNING'],
            'NOTICE': cls.__constants['NOTICE'],
            'INFO': cls.__constants['INFO'],
            'DEBUG': cls.__constants['DEBUG'],
        }

    @property
    def EMERGENCY(self):
        """Message constant 'EMERGENCY'."""
        return Metaclass_StatusText.__constants['EMERGENCY']

    @property
    def ALERT(self):
        """Message constant 'ALERT'."""
        return Metaclass_StatusText.__constants['ALERT']

    @property
    def CRITICAL(self):
        """Message constant 'CRITICAL'."""
        return Metaclass_StatusText.__constants['CRITICAL']

    @property
    def ERROR(self):
        """Message constant 'ERROR'."""
        return Metaclass_StatusText.__constants['ERROR']

    @property
    def WARNING(self):
        """Message constant 'WARNING'."""
        return Metaclass_StatusText.__constants['WARNING']

    @property
    def NOTICE(self):
        """Message constant 'NOTICE'."""
        return Metaclass_StatusText.__constants['NOTICE']

    @property
    def INFO(self):
        """Message constant 'INFO'."""
        return Metaclass_StatusText.__constants['INFO']

    @property
    def DEBUG(self):
        """Message constant 'DEBUG'."""
        return Metaclass_StatusText.__constants['DEBUG']


class StatusText(metaclass=Metaclass_StatusText):
    """
    Message class 'StatusText'.

    Constants:
      EMERGENCY
      ALERT
      CRITICAL
      ERROR
      WARNING
      NOTICE
      INFO
      DEBUG
    """

    __slots__ = [
        '_header',
        '_severity',
        '_text',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'severity': 'uint8',
        'text': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.severity = kwargs.get('severity', int())
        self.text = kwargs.get('text', str())

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
        if self.severity != other.severity:
            return False
        if self.text != other.text:
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
    def severity(self):
        """Message field 'severity'."""
        return self._severity

    @severity.setter
    def severity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'severity' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'severity' field must be an unsigned integer in [0, 255]"
        self._severity = value

    @builtins.property
    def text(self):
        """Message field 'text'."""
        return self._text

    @text.setter
    def text(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'text' field must be of type 'str'"
        self._text = value
