# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:srv/StreamRate.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StreamRate_Request(type):
    """Metaclass of message 'StreamRate_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STREAM_ALL': 0,
        'STREAM_RAW_SENSORS': 1,
        'STREAM_EXTENDED_STATUS': 2,
        'STREAM_RC_CHANNELS': 3,
        'STREAM_RAW_CONTROLLER': 4,
        'STREAM_POSITION': 6,
        'STREAM_EXTRA1': 10,
        'STREAM_EXTRA2': 11,
        'STREAM_EXTRA3': 12,
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
                'mavros_msgs.srv.StreamRate_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__stream_rate__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__stream_rate__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__stream_rate__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__stream_rate__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__stream_rate__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STREAM_ALL': cls.__constants['STREAM_ALL'],
            'STREAM_RAW_SENSORS': cls.__constants['STREAM_RAW_SENSORS'],
            'STREAM_EXTENDED_STATUS': cls.__constants['STREAM_EXTENDED_STATUS'],
            'STREAM_RC_CHANNELS': cls.__constants['STREAM_RC_CHANNELS'],
            'STREAM_RAW_CONTROLLER': cls.__constants['STREAM_RAW_CONTROLLER'],
            'STREAM_POSITION': cls.__constants['STREAM_POSITION'],
            'STREAM_EXTRA1': cls.__constants['STREAM_EXTRA1'],
            'STREAM_EXTRA2': cls.__constants['STREAM_EXTRA2'],
            'STREAM_EXTRA3': cls.__constants['STREAM_EXTRA3'],
        }

    @property
    def STREAM_ALL(self):
        """Message constant 'STREAM_ALL'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_ALL']

    @property
    def STREAM_RAW_SENSORS(self):
        """Message constant 'STREAM_RAW_SENSORS'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_RAW_SENSORS']

    @property
    def STREAM_EXTENDED_STATUS(self):
        """Message constant 'STREAM_EXTENDED_STATUS'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_EXTENDED_STATUS']

    @property
    def STREAM_RC_CHANNELS(self):
        """Message constant 'STREAM_RC_CHANNELS'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_RC_CHANNELS']

    @property
    def STREAM_RAW_CONTROLLER(self):
        """Message constant 'STREAM_RAW_CONTROLLER'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_RAW_CONTROLLER']

    @property
    def STREAM_POSITION(self):
        """Message constant 'STREAM_POSITION'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_POSITION']

    @property
    def STREAM_EXTRA1(self):
        """Message constant 'STREAM_EXTRA1'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_EXTRA1']

    @property
    def STREAM_EXTRA2(self):
        """Message constant 'STREAM_EXTRA2'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_EXTRA2']

    @property
    def STREAM_EXTRA3(self):
        """Message constant 'STREAM_EXTRA3'."""
        return Metaclass_StreamRate_Request.__constants['STREAM_EXTRA3']


class StreamRate_Request(metaclass=Metaclass_StreamRate_Request):
    """
    Message class 'StreamRate_Request'.

    Constants:
      STREAM_ALL
      STREAM_RAW_SENSORS
      STREAM_EXTENDED_STATUS
      STREAM_RC_CHANNELS
      STREAM_RAW_CONTROLLER
      STREAM_POSITION
      STREAM_EXTRA1
      STREAM_EXTRA2
      STREAM_EXTRA3
    """

    __slots__ = [
        '_stream_id',
        '_message_rate',
        '_on_off',
    ]

    _fields_and_field_types = {
        'stream_id': 'uint8',
        'message_rate': 'uint16',
        'on_off': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.stream_id = kwargs.get('stream_id', int())
        self.message_rate = kwargs.get('message_rate', int())
        self.on_off = kwargs.get('on_off', bool())

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
        if self.stream_id != other.stream_id:
            return False
        if self.message_rate != other.message_rate:
            return False
        if self.on_off != other.on_off:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stream_id(self):
        """Message field 'stream_id'."""
        return self._stream_id

    @stream_id.setter
    def stream_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'stream_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'stream_id' field must be an unsigned integer in [0, 255]"
        self._stream_id = value

    @builtins.property
    def message_rate(self):
        """Message field 'message_rate'."""
        return self._message_rate

    @message_rate.setter
    def message_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'message_rate' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'message_rate' field must be an unsigned integer in [0, 65535]"
        self._message_rate = value

    @builtins.property
    def on_off(self):
        """Message field 'on_off'."""
        return self._on_off

    @on_off.setter
    def on_off(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'on_off' field must be of type 'bool'"
        self._on_off = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_StreamRate_Response(type):
    """Metaclass of message 'StreamRate_Response'."""

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
                'mavros_msgs.srv.StreamRate_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__stream_rate__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__stream_rate__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__stream_rate__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__stream_rate__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__stream_rate__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StreamRate_Response(metaclass=Metaclass_StreamRate_Response):
    """Message class 'StreamRate_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_StreamRate(type):
    """Metaclass of service 'StreamRate'."""

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
                'mavros_msgs.srv.StreamRate')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__stream_rate

            from mavros_msgs.srv import _stream_rate
            if _stream_rate.Metaclass_StreamRate_Request._TYPE_SUPPORT is None:
                _stream_rate.Metaclass_StreamRate_Request.__import_type_support__()
            if _stream_rate.Metaclass_StreamRate_Response._TYPE_SUPPORT is None:
                _stream_rate.Metaclass_StreamRate_Response.__import_type_support__()


class StreamRate(metaclass=Metaclass_StreamRate):
    from mavros_msgs.srv._stream_rate import StreamRate_Request as Request
    from mavros_msgs.srv._stream_rate import StreamRate_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
