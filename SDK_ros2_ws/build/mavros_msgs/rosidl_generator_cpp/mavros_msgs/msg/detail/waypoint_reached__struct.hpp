// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/WaypointReached.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__WaypointReached __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__WaypointReached __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WaypointReached_
{
  using Type = WaypointReached_<ContainerAllocator>;

  explicit WaypointReached_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wp_seq = 0;
    }
  }

  explicit WaypointReached_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wp_seq = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _wp_seq_type =
    uint16_t;
  _wp_seq_type wp_seq;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__wp_seq(
    const uint16_t & _arg)
  {
    this->wp_seq = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::WaypointReached_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::WaypointReached_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::WaypointReached_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::WaypointReached_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__WaypointReached
    std::shared_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__WaypointReached
    std::shared_ptr<mavros_msgs::msg::WaypointReached_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointReached_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->wp_seq != other.wp_seq) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointReached_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointReached_

// alias to use template instance with default allocator
using WaypointReached =
  mavros_msgs::msg::WaypointReached_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__STRUCT_HPP_
