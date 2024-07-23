// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/MountConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__MountConfigure_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__MountConfigure_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MountConfigure_Request_
{
  using Type = MountConfigure_Request_<ContainerAllocator>;

  explicit MountConfigure_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->stabilize_roll = false;
      this->stabilize_pitch = false;
      this->stabilize_yaw = false;
      this->roll_input = 0;
      this->pitch_input = 0;
      this->yaw_input = 0;
    }
  }

  explicit MountConfigure_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->stabilize_roll = false;
      this->stabilize_pitch = false;
      this->stabilize_yaw = false;
      this->roll_input = 0;
      this->pitch_input = 0;
      this->yaw_input = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _stabilize_roll_type =
    bool;
  _stabilize_roll_type stabilize_roll;
  using _stabilize_pitch_type =
    bool;
  _stabilize_pitch_type stabilize_pitch;
  using _stabilize_yaw_type =
    bool;
  _stabilize_yaw_type stabilize_yaw;
  using _roll_input_type =
    uint8_t;
  _roll_input_type roll_input;
  using _pitch_input_type =
    uint8_t;
  _pitch_input_type pitch_input;
  using _yaw_input_type =
    uint8_t;
  _yaw_input_type yaw_input;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__stabilize_roll(
    const bool & _arg)
  {
    this->stabilize_roll = _arg;
    return *this;
  }
  Type & set__stabilize_pitch(
    const bool & _arg)
  {
    this->stabilize_pitch = _arg;
    return *this;
  }
  Type & set__stabilize_yaw(
    const bool & _arg)
  {
    this->stabilize_yaw = _arg;
    return *this;
  }
  Type & set__roll_input(
    const uint8_t & _arg)
  {
    this->roll_input = _arg;
    return *this;
  }
  Type & set__pitch_input(
    const uint8_t & _arg)
  {
    this->pitch_input = _arg;
    return *this;
  }
  Type & set__yaw_input(
    const uint8_t & _arg)
  {
    this->yaw_input = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MODE_RETRACT =
    0u;
  static constexpr uint8_t MODE_NEUTRAL =
    1u;
  static constexpr uint8_t MODE_MAVLINK_TARGETING =
    2u;
  static constexpr uint8_t MODE_RC_TARGETING =
    3u;
  static constexpr uint8_t MODE_GPS_POINT =
    4u;
  static constexpr uint8_t INPUT_ANGLE_BODY_FRAME =
    0u;
  static constexpr uint8_t INPUT_ANGULAR_RATE =
    1u;
  static constexpr uint8_t INPUT_ANGLE_ABSOLUTE_FRAME =
    2u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__MountConfigure_Request
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__MountConfigure_Request
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MountConfigure_Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->stabilize_roll != other.stabilize_roll) {
      return false;
    }
    if (this->stabilize_pitch != other.stabilize_pitch) {
      return false;
    }
    if (this->stabilize_yaw != other.stabilize_yaw) {
      return false;
    }
    if (this->roll_input != other.roll_input) {
      return false;
    }
    if (this->pitch_input != other.pitch_input) {
      return false;
    }
    if (this->yaw_input != other.yaw_input) {
      return false;
    }
    return true;
  }
  bool operator!=(const MountConfigure_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MountConfigure_Request_

// alias to use template instance with default allocator
using MountConfigure_Request =
  mavros_msgs::srv::MountConfigure_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::MODE_RETRACT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::MODE_NEUTRAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::MODE_MAVLINK_TARGETING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::MODE_RC_TARGETING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::MODE_GPS_POINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::INPUT_ANGLE_BODY_FRAME;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::INPUT_ANGULAR_RATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MountConfigure_Request_<ContainerAllocator>::INPUT_ANGLE_ABSOLUTE_FRAME;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__MountConfigure_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__MountConfigure_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MountConfigure_Response_
{
  using Type = MountConfigure_Response_<ContainerAllocator>;

  explicit MountConfigure_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  explicit MountConfigure_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _result_type =
    uint8_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__MountConfigure_Response
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__MountConfigure_Response
    std::shared_ptr<mavros_msgs::srv::MountConfigure_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MountConfigure_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const MountConfigure_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MountConfigure_Response_

// alias to use template instance with default allocator
using MountConfigure_Response =
  mavros_msgs::srv::MountConfigure_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct MountConfigure
{
  using Request = mavros_msgs::srv::MountConfigure_Request;
  using Response = mavros_msgs::srv::MountConfigure_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__STRUCT_HPP_
