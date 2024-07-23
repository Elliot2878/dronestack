// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/CommandCode.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__COMMAND_CODE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__COMMAND_CODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__CommandCode __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__CommandCode __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CommandCode_
{
  using Type = CommandCode_<ContainerAllocator>;

  explicit CommandCode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit CommandCode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations
  static constexpr uint16_t AIRFRAME_CONFIGURATION =
    2520u;
  static constexpr uint16_t ARM_AUTHORIZATION_REQUEST =
    3001u;
  static constexpr uint16_t CAMERA_TRACK_POINT =
    2004u;
  static constexpr uint16_t CAMERA_TRACK_RECTANGLE =
    2005u;
  static constexpr uint16_t CAMERA_STOP_TRACKING =
    2010u;
  static constexpr uint16_t CAN_FORWARD =
    32000u;
  static constexpr uint16_t COMPONENT_ARM_DISARM =
    400u;
  static constexpr uint16_t CONDITION_DELAY =
    112u;
  static constexpr uint16_t CONDITION_CHANGE_ALT =
    113u;
  static constexpr uint16_t CONDITION_DISTANCE =
    114u;
  static constexpr uint16_t CONDITION_YAW =
    115u;
  static constexpr uint16_t CONDITION_LAST =
    159u;
  static constexpr uint16_t CONTROL_HIGH_LATENCY =
    2600u;
  static constexpr uint16_t DO_FOLLOW =
    32u;
  static constexpr uint16_t DO_FOLLOW_REPOSITION =
    33u;
  static constexpr uint16_t DO_SET_MODE =
    176u;
  static constexpr uint16_t DO_JUMP =
    177u;
  static constexpr uint16_t DO_CHANGE_SPEED =
    178u;
  static constexpr uint16_t DO_SET_HOME =
    179u;
  static constexpr uint16_t DO_SET_PARAMETER =
    180u;
  static constexpr uint16_t DO_SET_RELAY =
    181u;
  static constexpr uint16_t DO_REPEAT_RELAY =
    182u;
  static constexpr uint16_t DO_SET_SERVO =
    183u;
  static constexpr uint16_t DO_REPEAT_SERVO =
    184u;
  static constexpr uint16_t DO_FLIGHTTERMINATION =
    185u;
  static constexpr uint16_t DO_CHANGE_ALTITUDE =
    186u;
  static constexpr uint16_t DO_LAND_START =
    189u;
  static constexpr uint16_t DO_RALLY_LAND =
    190u;
  static constexpr uint16_t DO_GO_AROUND =
    191u;
  static constexpr uint16_t DO_REPOSITION =
    192u;
  static constexpr uint16_t DO_PAUSE_CONTINUE =
    193u;
  static constexpr uint16_t DO_SET_REVERSE =
    194u;
  static constexpr uint16_t DO_SET_ROI_LOCATION =
    195u;
  static constexpr uint16_t DO_SET_ROI_WPNEXT_OFFSET =
    196u;
  static constexpr uint16_t DO_SET_ROI_NONE =
    197u;
  static constexpr uint16_t DO_SET_ROI_SYSID =
    198u;
  static constexpr uint16_t DO_CONTROL_VIDEO =
    200u;
  static constexpr uint16_t DO_SET_ROI =
    201u;
  static constexpr uint16_t DO_DIGICAM_CONFIGURE =
    202u;
  static constexpr uint16_t DO_DIGICAM_CONTROL =
    203u;
  static constexpr uint16_t DO_MOUNT_CONFIGURE =
    204u;
  static constexpr uint16_t DO_MOUNT_CONTROL =
    205u;
  static constexpr uint16_t DO_SET_CAM_TRIGG_DIST =
    206u;
  static constexpr uint16_t DO_FENCE_ENABLE =
    207u;
  static constexpr uint16_t DO_PARACHUTE =
    208u;
  static constexpr uint16_t DO_MOTOR_TEST =
    209u;
  static constexpr uint16_t DO_INVERTED_FLIGHT =
    210u;
  static constexpr uint16_t DO_GRIPPER =
    211u;
  static constexpr uint16_t DO_AUTOTUNE_ENABLE =
    212u;
  static constexpr uint16_t DO_SET_CAM_TRIGG_INTERVAL =
    214u;
  static constexpr uint16_t DO_MOUNT_CONTROL_QUAT =
    220u;
  static constexpr uint16_t DO_GUIDED_MASTER =
    221u;
  static constexpr uint16_t DO_GUIDED_LIMITS =
    222u;
  static constexpr uint16_t DO_ENGINE_CONTROL =
    223u;
  static constexpr uint16_t DO_SET_MISSION_CURRENT =
    224u;
  static constexpr uint16_t DO_LAST =
    240u;
  static constexpr uint16_t DO_JUMP_TAG =
    601u;
  static constexpr uint16_t DO_GIMBAL_MANAGER_PITCHYAW =
    1000u;
  static constexpr uint16_t DO_GIMBAL_MANAGER_CONFIGURE =
    1001u;
  static constexpr uint16_t DO_TRIGGER_CONTROL =
    2003u;
  static constexpr uint16_t DO_VTOL_TRANSITION =
    3000u;
  static constexpr uint16_t DO_ADSB_OUT_IDENT =
    10001u;
  static constexpr uint16_t DO_WINCH =
    42600u;
  static constexpr uint16_t FIXED_MAG_CAL_YAW =
    42006u;
  static constexpr uint16_t GET_HOME_POSITION =
    410u;
  static constexpr uint16_t GET_MESSAGE_INTERVAL =
    510u;
  static constexpr uint16_t IMAGE_START_CAPTURE =
    2000u;
  static constexpr uint16_t IMAGE_STOP_CAPTURE =
    2001u;
  static constexpr uint16_t JUMP_TAG =
    600u;
  static constexpr uint16_t LOGGING_START =
    2510u;
  static constexpr uint16_t LOGGING_STOP =
    2511u;
  static constexpr uint16_t MISSION_START =
    300u;
  static constexpr uint16_t NAV_WAYPOINT =
    16u;
  static constexpr uint16_t NAV_LOITER_UNLIM =
    17u;
  static constexpr uint16_t NAV_LOITER_TURNS =
    18u;
  static constexpr uint16_t NAV_LOITER_TIME =
    19u;
  static constexpr uint16_t NAV_RETURN_TO_LAUNCH =
    20u;
  static constexpr uint16_t NAV_LAND =
    21u;
  static constexpr uint16_t NAV_TAKEOFF =
    22u;
  static constexpr uint16_t NAV_LAND_LOCAL =
    23u;
  static constexpr uint16_t NAV_TAKEOFF_LOCAL =
    24u;
  static constexpr uint16_t NAV_FOLLOW =
    25u;
  static constexpr uint16_t NAV_CONTINUE_AND_CHANGE_ALT =
    30u;
  static constexpr uint16_t NAV_LOITER_TO_ALT =
    31u;
  static constexpr uint16_t NAV_ROI =
    80u;
  static constexpr uint16_t NAV_PATHPLANNING =
    81u;
  static constexpr uint16_t NAV_SPLINE_WAYPOINT =
    82u;
  static constexpr uint16_t NAV_VTOL_TAKEOFF =
    84u;
  static constexpr uint16_t NAV_VTOL_LAND =
    85u;
  static constexpr uint16_t NAV_GUIDED_ENABLE =
    92u;
  static constexpr uint16_t NAV_DELAY =
    93u;
  static constexpr uint16_t NAV_PAYLOAD_PLACE =
    94u;
  static constexpr uint16_t NAV_LAST =
    95u;
  static constexpr uint16_t NAV_SET_YAW_SPEED =
    213u;
  static constexpr uint16_t NAV_FENCE_RETURN_POINT =
    5000u;
  static constexpr uint16_t NAV_FENCE_POLYGON_VERTEX_INCLUSION =
    5001u;
  static constexpr uint16_t NAV_FENCE_POLYGON_VERTEX_EXCLUSION =
    5002u;
  static constexpr uint16_t NAV_FENCE_CIRCLE_INCLUSION =
    5003u;
  static constexpr uint16_t NAV_FENCE_CIRCLE_EXCLUSION =
    5004u;
  static constexpr uint16_t NAV_RALLY_POINT =
    5100u;
  static constexpr uint16_t OBLIQUE_SURVEY =
    260u;
  static constexpr uint16_t OVERRIDE_GOTO =
    252u;
  static constexpr uint16_t PANORAMA_CREATE =
    2800u;
  static constexpr uint16_t PAYLOAD_PREPARE_DEPLOY =
    30001u;
  static constexpr uint16_t PAYLOAD_CONTROL_DEPLOY =
    30002u;
  static constexpr uint16_t PREFLIGHT_CALIBRATION =
    241u;
  static constexpr uint16_t PREFLIGHT_SET_SENSOR_OFFSETS =
    242u;
  static constexpr uint16_t PREFLIGHT_UAVCAN =
    243u;
  static constexpr uint16_t PREFLIGHT_STORAGE =
    245u;
  static constexpr uint16_t PREFLIGHT_REBOOT_SHUTDOWN =
    246u;
  static constexpr uint16_t REQUEST_MESSAGE =
    512u;
  static constexpr uint16_t REQUEST_PROTOCOL_VERSION =
    519u;
  static constexpr uint16_t REQUEST_AUTOPILOT_CAPABILITIES =
    520u;
  static constexpr uint16_t REQUEST_CAMERA_INFORMATION =
    521u;
  static constexpr uint16_t REQUEST_CAMERA_SETTINGS =
    522u;
  static constexpr uint16_t REQUEST_STORAGE_INFORMATION =
    525u;
  static constexpr uint16_t REQUEST_CAMERA_CAPTURE_STATUS =
    527u;
  static constexpr uint16_t REQUEST_FLIGHT_INFORMATION =
    528u;
  static constexpr uint16_t REQUEST_VIDEO_STREAM_INFORMATION =
    2504u;
  static constexpr uint16_t REQUEST_VIDEO_STREAM_STATUS =
    2505u;
  static constexpr uint16_t RESET_CAMERA_SETTINGS =
    529u;
  static constexpr uint16_t RUN_PREARM_CHECKS =
    401u;
  static constexpr uint16_t SET_MESSAGE_INTERVAL =
    511u;
  static constexpr uint16_t SET_CAMERA_MODE =
    530u;
  static constexpr uint16_t SET_CAMERA_ZOOM =
    531u;
  static constexpr uint16_t SET_CAMERA_FOCUS =
    532u;
  static constexpr uint16_t SET_GUIDED_SUBMODE_STANDARD =
    4000u;
  static constexpr uint16_t SET_GUIDED_SUBMODE_CIRCLE =
    4001u;
  static constexpr uint16_t START_RX_PAIR =
    500u;
  static constexpr uint16_t STORAGE_FORMAT =
    526u;
  static constexpr uint16_t UAVCAN_GET_NODE_INFO =
    5200u;
  static constexpr uint16_t VIDEO_START_CAPTURE =
    2500u;
  static constexpr uint16_t VIDEO_STOP_CAPTURE =
    2501u;
  static constexpr uint16_t VIDEO_START_STREAMING =
    2502u;
  static constexpr uint16_t VIDEO_STOP_STREAMING =
    2503u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::CommandCode_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::CommandCode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CommandCode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CommandCode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__CommandCode
    std::shared_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__CommandCode
    std::shared_ptr<mavros_msgs::msg::CommandCode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CommandCode_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const CommandCode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CommandCode_

// alias to use template instance with default allocator
using CommandCode =
  mavros_msgs::msg::CommandCode_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::AIRFRAME_CONFIGURATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::ARM_AUTHORIZATION_REQUEST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CAMERA_TRACK_POINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CAMERA_TRACK_RECTANGLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CAMERA_STOP_TRACKING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CAN_FORWARD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::COMPONENT_ARM_DISARM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CONDITION_DELAY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CONDITION_CHANGE_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CONDITION_DISTANCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CONDITION_YAW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CONDITION_LAST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::CONTROL_HIGH_LATENCY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_FOLLOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_FOLLOW_REPOSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_JUMP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_CHANGE_SPEED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_HOME;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_PARAMETER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_RELAY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_REPEAT_RELAY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_SERVO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_REPEAT_SERVO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_FLIGHTTERMINATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_CHANGE_ALTITUDE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_LAND_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_RALLY_LAND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_GO_AROUND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_REPOSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_PAUSE_CONTINUE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_REVERSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_ROI_LOCATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_ROI_WPNEXT_OFFSET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_ROI_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_ROI_SYSID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_CONTROL_VIDEO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_ROI;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_DIGICAM_CONFIGURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_DIGICAM_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_MOUNT_CONFIGURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_MOUNT_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_CAM_TRIGG_DIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_FENCE_ENABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_PARACHUTE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_MOTOR_TEST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_INVERTED_FLIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_GRIPPER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_AUTOTUNE_ENABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_CAM_TRIGG_INTERVAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_MOUNT_CONTROL_QUAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_GUIDED_MASTER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_GUIDED_LIMITS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_ENGINE_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_SET_MISSION_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_LAST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_JUMP_TAG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_GIMBAL_MANAGER_PITCHYAW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_GIMBAL_MANAGER_CONFIGURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_TRIGGER_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_VTOL_TRANSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_ADSB_OUT_IDENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::DO_WINCH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::FIXED_MAG_CAL_YAW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::GET_HOME_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::GET_MESSAGE_INTERVAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::IMAGE_START_CAPTURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::IMAGE_STOP_CAPTURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::JUMP_TAG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::LOGGING_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::LOGGING_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::MISSION_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_WAYPOINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LOITER_UNLIM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LOITER_TURNS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LOITER_TIME;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_RETURN_TO_LAUNCH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LAND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_TAKEOFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LAND_LOCAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_TAKEOFF_LOCAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_FOLLOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_CONTINUE_AND_CHANGE_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LOITER_TO_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_ROI;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_PATHPLANNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_SPLINE_WAYPOINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_VTOL_TAKEOFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_VTOL_LAND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_GUIDED_ENABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_DELAY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_PAYLOAD_PLACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_LAST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_SET_YAW_SPEED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_FENCE_RETURN_POINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_FENCE_POLYGON_VERTEX_INCLUSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_FENCE_CIRCLE_INCLUSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_FENCE_CIRCLE_EXCLUSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::NAV_RALLY_POINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::OBLIQUE_SURVEY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::OVERRIDE_GOTO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PANORAMA_CREATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PAYLOAD_PREPARE_DEPLOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PAYLOAD_CONTROL_DEPLOY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PREFLIGHT_CALIBRATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PREFLIGHT_SET_SENSOR_OFFSETS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PREFLIGHT_UAVCAN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PREFLIGHT_STORAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::PREFLIGHT_REBOOT_SHUTDOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_MESSAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_PROTOCOL_VERSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_AUTOPILOT_CAPABILITIES;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_CAMERA_INFORMATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_CAMERA_SETTINGS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_STORAGE_INFORMATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_CAMERA_CAPTURE_STATUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_FLIGHT_INFORMATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_VIDEO_STREAM_INFORMATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::REQUEST_VIDEO_STREAM_STATUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::RESET_CAMERA_SETTINGS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::RUN_PREARM_CHECKS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::SET_MESSAGE_INTERVAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::SET_CAMERA_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::SET_CAMERA_ZOOM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::SET_CAMERA_FOCUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::SET_GUIDED_SUBMODE_STANDARD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::SET_GUIDED_SUBMODE_CIRCLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::START_RX_PAIR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::STORAGE_FORMAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::UAVCAN_GET_NODE_INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::VIDEO_START_CAPTURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::VIDEO_STOP_CAPTURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::VIDEO_START_STREAMING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t CommandCode_<ContainerAllocator>::VIDEO_STOP_STREAMING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__COMMAND_CODE__STRUCT_HPP_
