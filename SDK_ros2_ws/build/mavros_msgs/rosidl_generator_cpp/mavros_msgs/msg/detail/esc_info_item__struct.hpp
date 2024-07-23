// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ESCInfoItem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__ESCInfoItem __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ESCInfoItem __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ESCInfoItem_
{
  using Type = ESCInfoItem_<ContainerAllocator>;

  explicit ESCInfoItem_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->failure_flags = 0;
      this->error_count = 0ul;
      this->temperature = 0l;
    }
  }

  explicit ESCInfoItem_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->failure_flags = 0;
      this->error_count = 0ul;
      this->temperature = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _failure_flags_type =
    uint16_t;
  _failure_flags_type failure_flags;
  using _error_count_type =
    uint32_t;
  _error_count_type error_count;
  using _temperature_type =
    int32_t;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__failure_flags(
    const uint16_t & _arg)
  {
    this->failure_flags = _arg;
    return *this;
  }
  Type & set__error_count(
    const uint32_t & _arg)
  {
    this->error_count = _arg;
    return *this;
  }
  Type & set__temperature(
    const int32_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ESCInfoItem_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ESCInfoItem_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ESCInfoItem_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ESCInfoItem_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ESCInfoItem
    std::shared_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ESCInfoItem
    std::shared_ptr<mavros_msgs::msg::ESCInfoItem_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ESCInfoItem_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->failure_flags != other.failure_flags) {
      return false;
    }
    if (this->error_count != other.error_count) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const ESCInfoItem_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ESCInfoItem_

// alias to use template instance with default allocator
using ESCInfoItem =
  mavros_msgs::msg::ESCInfoItem_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__STRUCT_HPP_
