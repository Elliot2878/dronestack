// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/TimesyncStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__TimesyncStatus __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__TimesyncStatus __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TimesyncStatus_
{
  using Type = TimesyncStatus_<ContainerAllocator>;

  explicit TimesyncStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->remote_timestamp_ns = 0ull;
      this->observed_offset_ns = 0ll;
      this->estimated_offset_ns = 0ll;
      this->round_trip_time_ms = 0.0f;
    }
  }

  explicit TimesyncStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->remote_timestamp_ns = 0ull;
      this->observed_offset_ns = 0ll;
      this->estimated_offset_ns = 0ll;
      this->round_trip_time_ms = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _remote_timestamp_ns_type =
    uint64_t;
  _remote_timestamp_ns_type remote_timestamp_ns;
  using _observed_offset_ns_type =
    int64_t;
  _observed_offset_ns_type observed_offset_ns;
  using _estimated_offset_ns_type =
    int64_t;
  _estimated_offset_ns_type estimated_offset_ns;
  using _round_trip_time_ms_type =
    float;
  _round_trip_time_ms_type round_trip_time_ms;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__remote_timestamp_ns(
    const uint64_t & _arg)
  {
    this->remote_timestamp_ns = _arg;
    return *this;
  }
  Type & set__observed_offset_ns(
    const int64_t & _arg)
  {
    this->observed_offset_ns = _arg;
    return *this;
  }
  Type & set__estimated_offset_ns(
    const int64_t & _arg)
  {
    this->estimated_offset_ns = _arg;
    return *this;
  }
  Type & set__round_trip_time_ms(
    const float & _arg)
  {
    this->round_trip_time_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::TimesyncStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::TimesyncStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::TimesyncStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::TimesyncStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__TimesyncStatus
    std::shared_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__TimesyncStatus
    std::shared_ptr<mavros_msgs::msg::TimesyncStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimesyncStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->remote_timestamp_ns != other.remote_timestamp_ns) {
      return false;
    }
    if (this->observed_offset_ns != other.observed_offset_ns) {
      return false;
    }
    if (this->estimated_offset_ns != other.estimated_offset_ns) {
      return false;
    }
    if (this->round_trip_time_ms != other.round_trip_time_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimesyncStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimesyncStatus_

// alias to use template instance with default allocator
using TimesyncStatus =
  mavros_msgs::msg::TimesyncStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__STRUCT_HPP_
