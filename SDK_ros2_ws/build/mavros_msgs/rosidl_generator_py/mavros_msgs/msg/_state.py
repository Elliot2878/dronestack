# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/State.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_State(type):
    """Metaclass of message 'State'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_APM_PLANE_MANUAL': 'MANUAL',
        'MODE_APM_PLANE_CIRCLE': 'CIRCLE',
        'MODE_APM_PLANE_STABILIZE': 'STABILIZE',
        'MODE_APM_PLANE_TRAINING': 'TRAINING',
        'MODE_APM_PLANE_ACRO': 'ACRO',
        'MODE_APM_PLANE_FBWA': 'FBWA',
        'MODE_APM_PLANE_FBWB': 'FBWB',
        'MODE_APM_PLANE_CRUISE': 'CRUISE',
        'MODE_APM_PLANE_AUTOTUNE': 'AUTOTUNE',
        'MODE_APM_PLANE_AUTO': 'AUTO',
        'MODE_APM_PLANE_RTL': 'RTL',
        'MODE_APM_PLANE_LOITER': 'LOITER',
        'MODE_APM_PLANE_LAND': 'LAND',
        'MODE_APM_PLANE_GUIDED': 'GUIDED',
        'MODE_APM_PLANE_INITIALISING': 'INITIALISING',
        'MODE_APM_PLANE_QSTABILIZE': 'QSTABILIZE',
        'MODE_APM_PLANE_QHOVER': 'QHOVER',
        'MODE_APM_PLANE_QLOITER': 'QLOITER',
        'MODE_APM_PLANE_QLAND': 'QLAND',
        'MODE_APM_PLANE_QRTL': 'QRTL',
        'MODE_APM_COPTER_STABILIZE': 'STABILIZE',
        'MODE_APM_COPTER_ACRO': 'ACRO',
        'MODE_APM_COPTER_ALT_HOLD': 'ALT_HOLD',
        'MODE_APM_COPTER_AUTO': 'AUTO',
        'MODE_APM_COPTER_GUIDED': 'GUIDED',
        'MODE_APM_COPTER_LOITER': 'LOITER',
        'MODE_APM_COPTER_RTL': 'RTL',
        'MODE_APM_COPTER_CIRCLE': 'CIRCLE',
        'MODE_APM_COPTER_POSITION': 'POSITION',
        'MODE_APM_COPTER_LAND': 'LAND',
        'MODE_APM_COPTER_OF_LOITER': 'OF_LOITER',
        'MODE_APM_COPTER_DRIFT': 'DRIFT',
        'MODE_APM_COPTER_SPORT': 'SPORT',
        'MODE_APM_COPTER_FLIP': 'FLIP',
        'MODE_APM_COPTER_AUTOTUNE': 'AUTOTUNE',
        'MODE_APM_COPTER_POSHOLD': 'POSHOLD',
        'MODE_APM_COPTER_BRAKE': 'BRAKE',
        'MODE_APM_COPTER_THROW': 'THROW',
        'MODE_APM_COPTER_AVOID_ADSB': 'AVOID_ADSB',
        'MODE_APM_COPTER_GUIDED_NOGPS': 'GUIDED_NOGPS',
        'MODE_APM_ROVER_MANUAL': 'MANUAL',
        'MODE_APM_ROVER_LEARNING': 'LEARNING',
        'MODE_APM_ROVER_STEERING': 'STEERING',
        'MODE_APM_ROVER_HOLD': 'HOLD',
        'MODE_APM_ROVER_AUTO': 'AUTO',
        'MODE_APM_ROVER_RTL': 'RTL',
        'MODE_APM_ROVER_GUIDED': 'GUIDED',
        'MODE_APM_ROVER_INITIALISING': 'INITIALISING',
        'MODE_PX4_MANUAL': 'MANUAL',
        'MODE_PX4_ACRO': 'ACRO',
        'MODE_PX4_ALTITUDE': 'ALTCTL',
        'MODE_PX4_POSITION': 'POSCTL',
        'MODE_PX4_OFFBOARD': 'OFFBOARD',
        'MODE_PX4_STABILIZED': 'STABILIZED',
        'MODE_PX4_RATTITUDE': 'RATTITUDE',
        'MODE_PX4_MISSION': 'AUTO.MISSION',
        'MODE_PX4_LOITER': 'AUTO.LOITER',
        'MODE_PX4_RTL': 'AUTO.RTL',
        'MODE_PX4_LAND': 'AUTO.LAND',
        'MODE_PX4_RTGS': 'AUTO.RTGS',
        'MODE_PX4_READY': 'AUTO.READY',
        'MODE_PX4_TAKEOFF': 'AUTO.TAKEOFF',
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
                'mavros_msgs.msg.State')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_APM_PLANE_MANUAL': cls.__constants['MODE_APM_PLANE_MANUAL'],
            'MODE_APM_PLANE_CIRCLE': cls.__constants['MODE_APM_PLANE_CIRCLE'],
            'MODE_APM_PLANE_STABILIZE': cls.__constants['MODE_APM_PLANE_STABILIZE'],
            'MODE_APM_PLANE_TRAINING': cls.__constants['MODE_APM_PLANE_TRAINING'],
            'MODE_APM_PLANE_ACRO': cls.__constants['MODE_APM_PLANE_ACRO'],
            'MODE_APM_PLANE_FBWA': cls.__constants['MODE_APM_PLANE_FBWA'],
            'MODE_APM_PLANE_FBWB': cls.__constants['MODE_APM_PLANE_FBWB'],
            'MODE_APM_PLANE_CRUISE': cls.__constants['MODE_APM_PLANE_CRUISE'],
            'MODE_APM_PLANE_AUTOTUNE': cls.__constants['MODE_APM_PLANE_AUTOTUNE'],
            'MODE_APM_PLANE_AUTO': cls.__constants['MODE_APM_PLANE_AUTO'],
            'MODE_APM_PLANE_RTL': cls.__constants['MODE_APM_PLANE_RTL'],
            'MODE_APM_PLANE_LOITER': cls.__constants['MODE_APM_PLANE_LOITER'],
            'MODE_APM_PLANE_LAND': cls.__constants['MODE_APM_PLANE_LAND'],
            'MODE_APM_PLANE_GUIDED': cls.__constants['MODE_APM_PLANE_GUIDED'],
            'MODE_APM_PLANE_INITIALISING': cls.__constants['MODE_APM_PLANE_INITIALISING'],
            'MODE_APM_PLANE_QSTABILIZE': cls.__constants['MODE_APM_PLANE_QSTABILIZE'],
            'MODE_APM_PLANE_QHOVER': cls.__constants['MODE_APM_PLANE_QHOVER'],
            'MODE_APM_PLANE_QLOITER': cls.__constants['MODE_APM_PLANE_QLOITER'],
            'MODE_APM_PLANE_QLAND': cls.__constants['MODE_APM_PLANE_QLAND'],
            'MODE_APM_PLANE_QRTL': cls.__constants['MODE_APM_PLANE_QRTL'],
            'MODE_APM_COPTER_STABILIZE': cls.__constants['MODE_APM_COPTER_STABILIZE'],
            'MODE_APM_COPTER_ACRO': cls.__constants['MODE_APM_COPTER_ACRO'],
            'MODE_APM_COPTER_ALT_HOLD': cls.__constants['MODE_APM_COPTER_ALT_HOLD'],
            'MODE_APM_COPTER_AUTO': cls.__constants['MODE_APM_COPTER_AUTO'],
            'MODE_APM_COPTER_GUIDED': cls.__constants['MODE_APM_COPTER_GUIDED'],
            'MODE_APM_COPTER_LOITER': cls.__constants['MODE_APM_COPTER_LOITER'],
            'MODE_APM_COPTER_RTL': cls.__constants['MODE_APM_COPTER_RTL'],
            'MODE_APM_COPTER_CIRCLE': cls.__constants['MODE_APM_COPTER_CIRCLE'],
            'MODE_APM_COPTER_POSITION': cls.__constants['MODE_APM_COPTER_POSITION'],
            'MODE_APM_COPTER_LAND': cls.__constants['MODE_APM_COPTER_LAND'],
            'MODE_APM_COPTER_OF_LOITER': cls.__constants['MODE_APM_COPTER_OF_LOITER'],
            'MODE_APM_COPTER_DRIFT': cls.__constants['MODE_APM_COPTER_DRIFT'],
            'MODE_APM_COPTER_SPORT': cls.__constants['MODE_APM_COPTER_SPORT'],
            'MODE_APM_COPTER_FLIP': cls.__constants['MODE_APM_COPTER_FLIP'],
            'MODE_APM_COPTER_AUTOTUNE': cls.__constants['MODE_APM_COPTER_AUTOTUNE'],
            'MODE_APM_COPTER_POSHOLD': cls.__constants['MODE_APM_COPTER_POSHOLD'],
            'MODE_APM_COPTER_BRAKE': cls.__constants['MODE_APM_COPTER_BRAKE'],
            'MODE_APM_COPTER_THROW': cls.__constants['MODE_APM_COPTER_THROW'],
            'MODE_APM_COPTER_AVOID_ADSB': cls.__constants['MODE_APM_COPTER_AVOID_ADSB'],
            'MODE_APM_COPTER_GUIDED_NOGPS': cls.__constants['MODE_APM_COPTER_GUIDED_NOGPS'],
            'MODE_APM_ROVER_MANUAL': cls.__constants['MODE_APM_ROVER_MANUAL'],
            'MODE_APM_ROVER_LEARNING': cls.__constants['MODE_APM_ROVER_LEARNING'],
            'MODE_APM_ROVER_STEERING': cls.__constants['MODE_APM_ROVER_STEERING'],
            'MODE_APM_ROVER_HOLD': cls.__constants['MODE_APM_ROVER_HOLD'],
            'MODE_APM_ROVER_AUTO': cls.__constants['MODE_APM_ROVER_AUTO'],
            'MODE_APM_ROVER_RTL': cls.__constants['MODE_APM_ROVER_RTL'],
            'MODE_APM_ROVER_GUIDED': cls.__constants['MODE_APM_ROVER_GUIDED'],
            'MODE_APM_ROVER_INITIALISING': cls.__constants['MODE_APM_ROVER_INITIALISING'],
            'MODE_PX4_MANUAL': cls.__constants['MODE_PX4_MANUAL'],
            'MODE_PX4_ACRO': cls.__constants['MODE_PX4_ACRO'],
            'MODE_PX4_ALTITUDE': cls.__constants['MODE_PX4_ALTITUDE'],
            'MODE_PX4_POSITION': cls.__constants['MODE_PX4_POSITION'],
            'MODE_PX4_OFFBOARD': cls.__constants['MODE_PX4_OFFBOARD'],
            'MODE_PX4_STABILIZED': cls.__constants['MODE_PX4_STABILIZED'],
            'MODE_PX4_RATTITUDE': cls.__constants['MODE_PX4_RATTITUDE'],
            'MODE_PX4_MISSION': cls.__constants['MODE_PX4_MISSION'],
            'MODE_PX4_LOITER': cls.__constants['MODE_PX4_LOITER'],
            'MODE_PX4_RTL': cls.__constants['MODE_PX4_RTL'],
            'MODE_PX4_LAND': cls.__constants['MODE_PX4_LAND'],
            'MODE_PX4_RTGS': cls.__constants['MODE_PX4_RTGS'],
            'MODE_PX4_READY': cls.__constants['MODE_PX4_READY'],
            'MODE_PX4_TAKEOFF': cls.__constants['MODE_PX4_TAKEOFF'],
        }

    @property
    def MODE_APM_PLANE_MANUAL(self):
        """Message constant 'MODE_APM_PLANE_MANUAL'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_MANUAL']

    @property
    def MODE_APM_PLANE_CIRCLE(self):
        """Message constant 'MODE_APM_PLANE_CIRCLE'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_CIRCLE']

    @property
    def MODE_APM_PLANE_STABILIZE(self):
        """Message constant 'MODE_APM_PLANE_STABILIZE'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_STABILIZE']

    @property
    def MODE_APM_PLANE_TRAINING(self):
        """Message constant 'MODE_APM_PLANE_TRAINING'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_TRAINING']

    @property
    def MODE_APM_PLANE_ACRO(self):
        """Message constant 'MODE_APM_PLANE_ACRO'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_ACRO']

    @property
    def MODE_APM_PLANE_FBWA(self):
        """Message constant 'MODE_APM_PLANE_FBWA'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_FBWA']

    @property
    def MODE_APM_PLANE_FBWB(self):
        """Message constant 'MODE_APM_PLANE_FBWB'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_FBWB']

    @property
    def MODE_APM_PLANE_CRUISE(self):
        """Message constant 'MODE_APM_PLANE_CRUISE'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_CRUISE']

    @property
    def MODE_APM_PLANE_AUTOTUNE(self):
        """Message constant 'MODE_APM_PLANE_AUTOTUNE'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_AUTOTUNE']

    @property
    def MODE_APM_PLANE_AUTO(self):
        """Message constant 'MODE_APM_PLANE_AUTO'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_AUTO']

    @property
    def MODE_APM_PLANE_RTL(self):
        """Message constant 'MODE_APM_PLANE_RTL'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_RTL']

    @property
    def MODE_APM_PLANE_LOITER(self):
        """Message constant 'MODE_APM_PLANE_LOITER'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_LOITER']

    @property
    def MODE_APM_PLANE_LAND(self):
        """Message constant 'MODE_APM_PLANE_LAND'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_LAND']

    @property
    def MODE_APM_PLANE_GUIDED(self):
        """Message constant 'MODE_APM_PLANE_GUIDED'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_GUIDED']

    @property
    def MODE_APM_PLANE_INITIALISING(self):
        """Message constant 'MODE_APM_PLANE_INITIALISING'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_INITIALISING']

    @property
    def MODE_APM_PLANE_QSTABILIZE(self):
        """Message constant 'MODE_APM_PLANE_QSTABILIZE'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_QSTABILIZE']

    @property
    def MODE_APM_PLANE_QHOVER(self):
        """Message constant 'MODE_APM_PLANE_QHOVER'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_QHOVER']

    @property
    def MODE_APM_PLANE_QLOITER(self):
        """Message constant 'MODE_APM_PLANE_QLOITER'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_QLOITER']

    @property
    def MODE_APM_PLANE_QLAND(self):
        """Message constant 'MODE_APM_PLANE_QLAND'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_QLAND']

    @property
    def MODE_APM_PLANE_QRTL(self):
        """Message constant 'MODE_APM_PLANE_QRTL'."""
        return Metaclass_State.__constants['MODE_APM_PLANE_QRTL']

    @property
    def MODE_APM_COPTER_STABILIZE(self):
        """Message constant 'MODE_APM_COPTER_STABILIZE'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_STABILIZE']

    @property
    def MODE_APM_COPTER_ACRO(self):
        """Message constant 'MODE_APM_COPTER_ACRO'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_ACRO']

    @property
    def MODE_APM_COPTER_ALT_HOLD(self):
        """Message constant 'MODE_APM_COPTER_ALT_HOLD'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_ALT_HOLD']

    @property
    def MODE_APM_COPTER_AUTO(self):
        """Message constant 'MODE_APM_COPTER_AUTO'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_AUTO']

    @property
    def MODE_APM_COPTER_GUIDED(self):
        """Message constant 'MODE_APM_COPTER_GUIDED'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_GUIDED']

    @property
    def MODE_APM_COPTER_LOITER(self):
        """Message constant 'MODE_APM_COPTER_LOITER'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_LOITER']

    @property
    def MODE_APM_COPTER_RTL(self):
        """Message constant 'MODE_APM_COPTER_RTL'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_RTL']

    @property
    def MODE_APM_COPTER_CIRCLE(self):
        """Message constant 'MODE_APM_COPTER_CIRCLE'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_CIRCLE']

    @property
    def MODE_APM_COPTER_POSITION(self):
        """Message constant 'MODE_APM_COPTER_POSITION'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_POSITION']

    @property
    def MODE_APM_COPTER_LAND(self):
        """Message constant 'MODE_APM_COPTER_LAND'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_LAND']

    @property
    def MODE_APM_COPTER_OF_LOITER(self):
        """Message constant 'MODE_APM_COPTER_OF_LOITER'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_OF_LOITER']

    @property
    def MODE_APM_COPTER_DRIFT(self):
        """Message constant 'MODE_APM_COPTER_DRIFT'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_DRIFT']

    @property
    def MODE_APM_COPTER_SPORT(self):
        """Message constant 'MODE_APM_COPTER_SPORT'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_SPORT']

    @property
    def MODE_APM_COPTER_FLIP(self):
        """Message constant 'MODE_APM_COPTER_FLIP'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_FLIP']

    @property
    def MODE_APM_COPTER_AUTOTUNE(self):
        """Message constant 'MODE_APM_COPTER_AUTOTUNE'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_AUTOTUNE']

    @property
    def MODE_APM_COPTER_POSHOLD(self):
        """Message constant 'MODE_APM_COPTER_POSHOLD'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_POSHOLD']

    @property
    def MODE_APM_COPTER_BRAKE(self):
        """Message constant 'MODE_APM_COPTER_BRAKE'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_BRAKE']

    @property
    def MODE_APM_COPTER_THROW(self):
        """Message constant 'MODE_APM_COPTER_THROW'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_THROW']

    @property
    def MODE_APM_COPTER_AVOID_ADSB(self):
        """Message constant 'MODE_APM_COPTER_AVOID_ADSB'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_AVOID_ADSB']

    @property
    def MODE_APM_COPTER_GUIDED_NOGPS(self):
        """Message constant 'MODE_APM_COPTER_GUIDED_NOGPS'."""
        return Metaclass_State.__constants['MODE_APM_COPTER_GUIDED_NOGPS']

    @property
    def MODE_APM_ROVER_MANUAL(self):
        """Message constant 'MODE_APM_ROVER_MANUAL'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_MANUAL']

    @property
    def MODE_APM_ROVER_LEARNING(self):
        """Message constant 'MODE_APM_ROVER_LEARNING'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_LEARNING']

    @property
    def MODE_APM_ROVER_STEERING(self):
        """Message constant 'MODE_APM_ROVER_STEERING'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_STEERING']

    @property
    def MODE_APM_ROVER_HOLD(self):
        """Message constant 'MODE_APM_ROVER_HOLD'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_HOLD']

    @property
    def MODE_APM_ROVER_AUTO(self):
        """Message constant 'MODE_APM_ROVER_AUTO'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_AUTO']

    @property
    def MODE_APM_ROVER_RTL(self):
        """Message constant 'MODE_APM_ROVER_RTL'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_RTL']

    @property
    def MODE_APM_ROVER_GUIDED(self):
        """Message constant 'MODE_APM_ROVER_GUIDED'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_GUIDED']

    @property
    def MODE_APM_ROVER_INITIALISING(self):
        """Message constant 'MODE_APM_ROVER_INITIALISING'."""
        return Metaclass_State.__constants['MODE_APM_ROVER_INITIALISING']

    @property
    def MODE_PX4_MANUAL(self):
        """Message constant 'MODE_PX4_MANUAL'."""
        return Metaclass_State.__constants['MODE_PX4_MANUAL']

    @property
    def MODE_PX4_ACRO(self):
        """Message constant 'MODE_PX4_ACRO'."""
        return Metaclass_State.__constants['MODE_PX4_ACRO']

    @property
    def MODE_PX4_ALTITUDE(self):
        """Message constant 'MODE_PX4_ALTITUDE'."""
        return Metaclass_State.__constants['MODE_PX4_ALTITUDE']

    @property
    def MODE_PX4_POSITION(self):
        """Message constant 'MODE_PX4_POSITION'."""
        return Metaclass_State.__constants['MODE_PX4_POSITION']

    @property
    def MODE_PX4_OFFBOARD(self):
        """Message constant 'MODE_PX4_OFFBOARD'."""
        return Metaclass_State.__constants['MODE_PX4_OFFBOARD']

    @property
    def MODE_PX4_STABILIZED(self):
        """Message constant 'MODE_PX4_STABILIZED'."""
        return Metaclass_State.__constants['MODE_PX4_STABILIZED']

    @property
    def MODE_PX4_RATTITUDE(self):
        """Message constant 'MODE_PX4_RATTITUDE'."""
        return Metaclass_State.__constants['MODE_PX4_RATTITUDE']

    @property
    def MODE_PX4_MISSION(self):
        """Message constant 'MODE_PX4_MISSION'."""
        return Metaclass_State.__constants['MODE_PX4_MISSION']

    @property
    def MODE_PX4_LOITER(self):
        """Message constant 'MODE_PX4_LOITER'."""
        return Metaclass_State.__constants['MODE_PX4_LOITER']

    @property
    def MODE_PX4_RTL(self):
        """Message constant 'MODE_PX4_RTL'."""
        return Metaclass_State.__constants['MODE_PX4_RTL']

    @property
    def MODE_PX4_LAND(self):
        """Message constant 'MODE_PX4_LAND'."""
        return Metaclass_State.__constants['MODE_PX4_LAND']

    @property
    def MODE_PX4_RTGS(self):
        """Message constant 'MODE_PX4_RTGS'."""
        return Metaclass_State.__constants['MODE_PX4_RTGS']

    @property
    def MODE_PX4_READY(self):
        """Message constant 'MODE_PX4_READY'."""
        return Metaclass_State.__constants['MODE_PX4_READY']

    @property
    def MODE_PX4_TAKEOFF(self):
        """Message constant 'MODE_PX4_TAKEOFF'."""
        return Metaclass_State.__constants['MODE_PX4_TAKEOFF']


class State(metaclass=Metaclass_State):
    """
    Message class 'State'.

    Constants:
      MODE_APM_PLANE_MANUAL
      MODE_APM_PLANE_CIRCLE
      MODE_APM_PLANE_STABILIZE
      MODE_APM_PLANE_TRAINING
      MODE_APM_PLANE_ACRO
      MODE_APM_PLANE_FBWA
      MODE_APM_PLANE_FBWB
      MODE_APM_PLANE_CRUISE
      MODE_APM_PLANE_AUTOTUNE
      MODE_APM_PLANE_AUTO
      MODE_APM_PLANE_RTL
      MODE_APM_PLANE_LOITER
      MODE_APM_PLANE_LAND
      MODE_APM_PLANE_GUIDED
      MODE_APM_PLANE_INITIALISING
      MODE_APM_PLANE_QSTABILIZE
      MODE_APM_PLANE_QHOVER
      MODE_APM_PLANE_QLOITER
      MODE_APM_PLANE_QLAND
      MODE_APM_PLANE_QRTL
      MODE_APM_COPTER_STABILIZE
      MODE_APM_COPTER_ACRO
      MODE_APM_COPTER_ALT_HOLD
      MODE_APM_COPTER_AUTO
      MODE_APM_COPTER_GUIDED
      MODE_APM_COPTER_LOITER
      MODE_APM_COPTER_RTL
      MODE_APM_COPTER_CIRCLE
      MODE_APM_COPTER_POSITION
      MODE_APM_COPTER_LAND
      MODE_APM_COPTER_OF_LOITER
      MODE_APM_COPTER_DRIFT
      MODE_APM_COPTER_SPORT
      MODE_APM_COPTER_FLIP
      MODE_APM_COPTER_AUTOTUNE
      MODE_APM_COPTER_POSHOLD
      MODE_APM_COPTER_BRAKE
      MODE_APM_COPTER_THROW
      MODE_APM_COPTER_AVOID_ADSB
      MODE_APM_COPTER_GUIDED_NOGPS
      MODE_APM_ROVER_MANUAL
      MODE_APM_ROVER_LEARNING
      MODE_APM_ROVER_STEERING
      MODE_APM_ROVER_HOLD
      MODE_APM_ROVER_AUTO
      MODE_APM_ROVER_RTL
      MODE_APM_ROVER_GUIDED
      MODE_APM_ROVER_INITIALISING
      MODE_PX4_MANUAL
      MODE_PX4_ACRO
      MODE_PX4_ALTITUDE
      MODE_PX4_POSITION
      MODE_PX4_OFFBOARD
      MODE_PX4_STABILIZED
      MODE_PX4_RATTITUDE
      MODE_PX4_MISSION
      MODE_PX4_LOITER
      MODE_PX4_RTL
      MODE_PX4_LAND
      MODE_PX4_RTGS
      MODE_PX4_READY
      MODE_PX4_TAKEOFF
    """

    __slots__ = [
        '_header',
        '_connected',
        '_armed',
        '_guided',
        '_manual_input',
        '_mode',
        '_system_status',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'connected': 'boolean',
        'armed': 'boolean',
        'guided': 'boolean',
        'manual_input': 'boolean',
        'mode': 'string',
        'system_status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.connected = kwargs.get('connected', bool())
        self.armed = kwargs.get('armed', bool())
        self.guided = kwargs.get('guided', bool())
        self.manual_input = kwargs.get('manual_input', bool())
        self.mode = kwargs.get('mode', str())
        self.system_status = kwargs.get('system_status', int())

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
        if self.connected != other.connected:
            return False
        if self.armed != other.armed:
            return False
        if self.guided != other.guided:
            return False
        if self.manual_input != other.manual_input:
            return False
        if self.mode != other.mode:
            return False
        if self.system_status != other.system_status:
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
    def connected(self):
        """Message field 'connected'."""
        return self._connected

    @connected.setter
    def connected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'connected' field must be of type 'bool'"
        self._connected = value

    @builtins.property
    def armed(self):
        """Message field 'armed'."""
        return self._armed

    @armed.setter
    def armed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'armed' field must be of type 'bool'"
        self._armed = value

    @builtins.property
    def guided(self):
        """Message field 'guided'."""
        return self._guided

    @guided.setter
    def guided(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'guided' field must be of type 'bool'"
        self._guided = value

    @builtins.property
    def manual_input(self):
        """Message field 'manual_input'."""
        return self._manual_input

    @manual_input.setter
    def manual_input(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'manual_input' field must be of type 'bool'"
        self._manual_input = value

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
