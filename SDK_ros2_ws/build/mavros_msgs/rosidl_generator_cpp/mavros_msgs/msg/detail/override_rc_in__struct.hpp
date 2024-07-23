// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/OverrideRCIn.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__OverrideRCIn __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__OverrideRCIn __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OverrideRCIn_
{
  using Type = OverrideRCIn_<ContainerAllocator>;

  explicit OverrideRCIn_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint16_t, 18>::iterator, uint16_t>(this->channels.begin(), this->channels.end(), 0);
    }
  }

  explicit OverrideRCIn_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : channels(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint16_t, 18>::iterator, uint16_t>(this->channels.begin(), this->channels.end(), 0);
    }
  }

  // field types and members
  using _channels_type =
    std::array<uint16_t, 18>;
  _channels_type channels;

  // setters for named parameter idiom
  Type & set__channels(
    const std::array<uint16_t, 18> & _arg)
  {
    this->channels = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t CHAN_RELEASE =
    0u;
  static constexpr uint16_t CHAN_NOCHANGE =
    65535u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::OverrideRCIn_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::OverrideRCIn_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OverrideRCIn_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OverrideRCIn_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__OverrideRCIn
    std::shared_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__OverrideRCIn
    std::shared_ptr<mavros_msgs::msg::OverrideRCIn_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OverrideRCIn_ & other) const
  {
    if (this->channels != other.channels) {
      return false;
    }
    return true;
  }
  bool operator!=(const OverrideRCIn_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OverrideRCIn_

// alias to use template instance with default allocator
using OverrideRCIn =
  mavros_msgs::msg::OverrideRCIn_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t OverrideRCIn_<ContainerAllocator>::CHAN_RELEASE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t OverrideRCIn_<ContainerAllocator>::CHAN_NOCHANGE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__STRUCT_HPP_
