// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ESCTelemetry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__STRUCT_HPP_

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
// Member 'esc_telemetry'
#include "mavros_msgs/msg/detail/esc_telemetry_item__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__ESCTelemetry __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ESCTelemetry __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ESCTelemetry_
{
  using Type = ESCTelemetry_<ContainerAllocator>;

  explicit ESCTelemetry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ESCTelemetry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _esc_telemetry_type =
    std::vector<mavros_msgs::msg::ESCTelemetryItem_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mavros_msgs::msg::ESCTelemetryItem_<ContainerAllocator>>>;
  _esc_telemetry_type esc_telemetry;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__esc_telemetry(
    const std::vector<mavros_msgs::msg::ESCTelemetryItem_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mavros_msgs::msg::ESCTelemetryItem_<ContainerAllocator>>> & _arg)
  {
    this->esc_telemetry = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ESCTelemetry_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ESCTelemetry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ESCTelemetry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ESCTelemetry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ESCTelemetry
    std::shared_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ESCTelemetry
    std::shared_ptr<mavros_msgs::msg::ESCTelemetry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ESCTelemetry_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->esc_telemetry != other.esc_telemetry) {
      return false;
    }
    return true;
  }
  bool operator!=(const ESCTelemetry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ESCTelemetry_

// alias to use template instance with default allocator
using ESCTelemetry =
  mavros_msgs::msg::ESCTelemetry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__STRUCT_HPP_
