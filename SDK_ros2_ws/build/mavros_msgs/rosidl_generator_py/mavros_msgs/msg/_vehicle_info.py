# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/VehicleInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VehicleInfo(type):
    """Metaclass of message 'VehicleInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'HAVE_INFO_HEARTBEAT': 1,
        'HAVE_INFO_AUTOPILOT_VERSION': 2,
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
                'mavros_msgs.msg.VehicleInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__vehicle_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__vehicle_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__vehicle_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__vehicle_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__vehicle_info

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'HAVE_INFO_HEARTBEAT': cls.__constants['HAVE_INFO_HEARTBEAT'],
            'HAVE_INFO_AUTOPILOT_VERSION': cls.__constants['HAVE_INFO_AUTOPILOT_VERSION'],
        }

    @property
    def HAVE_INFO_HEARTBEAT(self):
        """Message constant 'HAVE_INFO_HEARTBEAT'."""
        return Metaclass_VehicleInfo.__constants['HAVE_INFO_HEARTBEAT']

    @property
    def HAVE_INFO_AUTOPILOT_VERSION(self):
        """Message constant 'HAVE_INFO_AUTOPILOT_VERSION'."""
        return Metaclass_VehicleInfo.__constants['HAVE_INFO_AUTOPILOT_VERSION']


class VehicleInfo(metaclass=Metaclass_VehicleInfo):
    """
    Message class 'VehicleInfo'.

    Constants:
      HAVE_INFO_HEARTBEAT
      HAVE_INFO_AUTOPILOT_VERSION
    """

    __slots__ = [
        '_header',
        '_available_info',
        '_sysid',
        '_compid',
        '_autopilot',
        '_type',
        '_system_status',
        '_base_mode',
        '_custom_mode',
        '_mode',
        '_mode_id',
        '_capabilities',
        '_flight_sw_version',
        '_middleware_sw_version',
        '_os_sw_version',
        '_board_version',
        '_flight_custom_version',
        '_vendor_id',
        '_product_id',
        '_uid',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'available_info': 'uint8',
        'sysid': 'uint8',
        'compid': 'uint8',
        'autopilot': 'uint8',
        'type': 'uint8',
        'system_status': 'uint8',
        'base_mode': 'uint8',
        'custom_mode': 'uint32',
        'mode': 'string',
        'mode_id': 'uint32',
        'capabilities': 'uint64',
        'flight_sw_version': 'uint32',
        'middleware_sw_version': 'uint32',
        'os_sw_version': 'uint32',
        'board_version': 'uint32',
        'flight_custom_version': 'string',
        'vendor_id': 'uint16',
        'product_id': 'uint16',
        'uid': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.available_info = kwargs.get('available_info', int())
        self.sysid = kwargs.get('sysid', int())
        self.compid = kwargs.get('compid', int())
        self.autopilot = kwargs.get('autopilot', int())
        self.type = kwargs.get('type', int())
        self.system_status = kwargs.get('system_status', int())
        self.base_mode = kwargs.get('base_mode', int())
        self.custom_mode = kwargs.get('custom_mode', int())
        self.mode = kwargs.get('mode', str())
        self.mode_id = kwargs.get('mode_id', int())
        self.capabilities = kwargs.get('capabilities', int())
        self.flight_sw_version = kwargs.get('flight_sw_version', int())
        self.middleware_sw_version = kwargs.get('middleware_sw_version', int())
        self.os_sw_version = kwargs.get('os_sw_version', int())
        self.board_version = kwargs.get('board_version', int())
        self.flight_custom_version = kwargs.get('flight_custom_version', str())
        self.vendor_id = kwargs.get('vendor_id', int())
        self.product_id = kwargs.get('product_id', int())
        self.uid = kwargs.get('uid', int())

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
        if self.available_info != other.available_info:
            return False
        if self.sysid != other.sysid:
            return False
        if self.compid != other.compid:
            return False
        if self.autopilot != other.autopilot:
            return False
        if self.type != other.type:
            return False
        if self.system_status != other.system_status:
            return False
        if self.base_mode != other.base_mode:
            return False
        if self.custom_mode != other.custom_mode:
            return False
        if self.mode != other.mode:
            return False
        if self.mode_id != other.mode_id:
            return False
        if self.capabilities != other.capabilities:
            return False
        if self.flight_sw_version != other.flight_sw_version:
            return False
        if self.middleware_sw_version != other.middleware_sw_version:
            return False
        if self.os_sw_version != other.os_sw_version:
            return False
        if self.board_version != other.board_version:
            return False
        if self.flight_custom_version != other.flight_custom_version:
            return False
        if self.vendor_id != other.vendor_id:
            return False
        if self.product_id != other.product_id:
            return False
        if self.uid != other.uid:
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
    def available_info(self):
        """Message field 'available_info'."""
        return self._available_info

    @available_info.setter
    def available_info(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'available_info' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'available_info' field must be an unsigned integer in [0, 255]"
        self._available_info = value

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

    @builtins.property
    def compid(self):
        """Message field 'compid'."""
        return self._compid

    @compid.setter
    def compid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compid' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compid' field must be an unsigned integer in [0, 255]"
        self._compid = value

    @builtins.property
    def autopilot(self):
        """Message field 'autopilot'."""
        return self._autopilot

    @autopilot.setter
    def autopilot(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'autopilot' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'autopilot' field must be an unsigned integer in [0, 255]"
        self._autopilot = value

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

    @builtins.property
    def system_status(self):
        """Message field 'system_status'."""
        return self._system_status

    @system_status.setter
    def system_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'system_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'system_status' field must be an unsigned integer in [0, 255]"
        self._system_status = value

    @builtins.property
    def base_mode(self):
        """Message field 'base_mode'."""
        return self._base_mode

    @base_mode.setter
    def base_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'base_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'base_mode' field must be an unsigned integer in [0, 255]"
        self._base_mode = value

    @builtins.property
    def custom_mode(self):
        """Message field 'custom_mode'."""
        return self._custom_mode

    @custom_mode.setter
    def custom_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'custom_mode' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'custom_mode' field must be an unsigned integer in [0, 4294967295]"
        self._custom_mode = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'mode' field must be of type 'str'"
        self._mode = value

    @builtins.property
    def mode_id(self):
        """Message field 'mode_id'."""
        return self._mode_id

    @mode_id.setter
    def mode_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'mode_id' field must be an unsigned integer in [0, 4294967295]"
        self._mode_id = value

    @builtins.property
    def capabilities(self):
        """Message field 'capabilities'."""
        return self._capabilities

    @capabilities.setter
    def capabilities(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'capabilities' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'capabilities' field must be an unsigned integer in [0, 18446744073709551615]"
        self._capabilities = value

    @builtins.property
    def flight_sw_version(self):
        """Message field 'flight_sw_version'."""
        return self._flight_sw_version

    @flight_sw_version.setter
    def flight_sw_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'flight_sw_version' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'flight_sw_version' field must be an unsigned integer in [0, 4294967295]"
        self._flight_sw_version = value

    @builtins.property
    def middleware_sw_version(self):
        """Message field 'middleware_sw_version'."""
        return self._middleware_sw_version

    @middleware_sw_version.setter
    def middleware_sw_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'middleware_sw_version' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'middleware_sw_version' field must be an unsigned integer in [0, 4294967295]"
        self._middleware_sw_version = value

    @builtins.property
    def os_sw_version(self):
        """Message field 'os_sw_version'."""
        return self._os_sw_version

    @os_sw_version.setter
    def os_sw_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'os_sw_version' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'os_sw_version' field must be an unsigned integer in [0, 4294967295]"
        self._os_sw_version = value

    @builtins.property
    def board_version(self):
        """Message field 'board_version'."""
        return self._board_version

    @board_version.setter
    def board_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'board_version' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'board_version' field must be an unsigned integer in [0, 4294967295]"
        self._board_version = value

    @builtins.property
    def flight_custom_version(self):
        """Message field 'flight_custom_version'."""
        return self._flight_custom_version

    @flight_custom_version.setter
    def flight_custom_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'flight_custom_version' field must be of type 'str'"
        self._flight_custom_version = value

    @builtins.property
    def vendor_id(self):
        """Message field 'vendor_id'."""
        return self._vendor_id

    @vendor_id.setter
    def vendor_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vendor_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'vendor_id' field must be an unsigned integer in [0, 65535]"
        self._vendor_id = value

    @builtins.property
    def product_id(self):
        """Message field 'product_id'."""
        return self._product_id

    @product_id.setter
    def product_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'product_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'product_id' field must be an unsigned integer in [0, 65535]"
        self._product_id = value

    @builtins.property
    def uid(self):
        """Message field 'uid'."""
        return self._uid

    @uid.setter
    def uid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'uid' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'uid' field must be an unsigned integer in [0, 18446744073709551615]"
        self._uid = value
