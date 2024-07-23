// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/CompanionProcessStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__CompanionProcessStatus __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__CompanionProcessStatus __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CompanionProcessStatus_
{
  using Type = CompanionProcessStatus_<ContainerAllocator>;

  explicit CompanionProcessStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->component = 0;
    }
  }

  explicit CompanionProcessStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->component = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _component_type =
    uint8_t;
  _component_type component;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__component(
    const uint8_t & _arg)
  {
    this->component = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MAV_STATE_UNINIT =
    0u;
  static constexpr uint8_t MAV_STATE_BOOT =
    1u;
  static constexpr uint8_t MAV_STATE_CALIBRATING =
    2u;
  static constexpr uint8_t MAV_STATE_STANDBY =
    3u;
  static constexpr uint8_t MAV_STATE_ACTIVE =
    4u;
  static constexpr uint8_t MAV_STATE_CRITICAL =
    5u;
  static constexpr uint8_t MAV_STATE_EMERGENCY =
    6u;
  static constexpr uint8_t MAV_STATE_POWEROFF =
    7u;
  static constexpr uint8_t MAV_STATE_FLIGHT_TERMINATION =
    8u;
  static constexpr uint8_t MAV_COMP_ID_OBSTACLE_AVOIDANCE =
    196u;
  static constexpr uint8_t MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY =
    197u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__CompanionProcessStatus
    std::shared_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__CompanionProcessStatus
    std::shared_ptr<mavros_msgs::msg::CompanionProcessStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CompanionProcessStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->component != other.component) {
      return false;
    }
    return true;
  }
  bool operator!=(const CompanionProcessStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CompanionProcessStatus_

// alias to use template instance with default allocator
using CompanionProcessStatus =
  mavros_msgs::msg::CompanionProcessStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_UNINIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_BOOT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_CALIBRATING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_STANDBY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_ACTIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_CRITICAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_EMERGENCY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_POWEROFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_STATE_FLIGHT_TERMINATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_COMP_ID_OBSTACLE_AVOIDANCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CompanionProcessStatus_<ContainerAllocator>::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__STRUCT_HPP_
