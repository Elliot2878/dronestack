// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ExtendedState.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__ExtendedState __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ExtendedState __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ExtendedState_
{
  using Type = ExtendedState_<ContainerAllocator>;

  explicit ExtendedState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vtol_state = 0;
      this->landed_state = 0;
    }
  }

  explicit ExtendedState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vtol_state = 0;
      this->landed_state = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vtol_state_type =
    uint8_t;
  _vtol_state_type vtol_state;
  using _landed_state_type =
    uint8_t;
  _landed_state_type landed_state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vtol_state(
    const uint8_t & _arg)
  {
    this->vtol_state = _arg;
    return *this;
  }
  Type & set__landed_state(
    const uint8_t & _arg)
  {
    this->landed_state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t VTOL_STATE_UNDEFINED =
    0u;
  static constexpr uint8_t VTOL_STATE_TRANSITION_TO_FW =
    1u;
  static constexpr uint8_t VTOL_STATE_TRANSITION_TO_MC =
    2u;
  static constexpr uint8_t VTOL_STATE_MC =
    3u;
  static constexpr uint8_t VTOL_STATE_FW =
    4u;
  static constexpr uint8_t LANDED_STATE_UNDEFINED =
    0u;
  static constexpr uint8_t LANDED_STATE_ON_GROUND =
    1u;
  static constexpr uint8_t LANDED_STATE_IN_AIR =
    2u;
  static constexpr uint8_t LANDED_STATE_TAKEOFF =
    3u;
  static constexpr uint8_t LANDED_STATE_LANDING =
    4u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ExtendedState_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ExtendedState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ExtendedState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ExtendedState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ExtendedState
    std::shared_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ExtendedState
    std::shared_ptr<mavros_msgs::msg::ExtendedState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExtendedState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vtol_state != other.vtol_state) {
      return false;
    }
    if (this->landed_state != other.landed_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExtendedState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExtendedState_

// alias to use template instance with default allocator
using ExtendedState =
  mavros_msgs::msg::ExtendedState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::VTOL_STATE_UNDEFINED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::VTOL_STATE_TRANSITION_TO_FW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::VTOL_STATE_TRANSITION_TO_MC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::VTOL_STATE_MC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::VTOL_STATE_FW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::LANDED_STATE_UNDEFINED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::LANDED_STATE_ON_GROUND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::LANDED_STATE_IN_AIR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::LANDED_STATE_TAKEOFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExtendedState_<ContainerAllocator>::LANDED_STATE_LANDING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__STRUCT_HPP_
