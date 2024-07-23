// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/LogEntry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__STRUCT_HPP_

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
// Member 'time_utc'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__LogEntry __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__LogEntry __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LogEntry_
{
  using Type = LogEntry_<ContainerAllocator>;

  explicit LogEntry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time_utc(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->num_logs = 0;
      this->last_log_num = 0;
      this->size = 0ul;
    }
  }

  explicit LogEntry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time_utc(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->num_logs = 0;
      this->last_log_num = 0;
      this->size = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint16_t;
  _id_type id;
  using _num_logs_type =
    uint16_t;
  _num_logs_type num_logs;
  using _last_log_num_type =
    uint16_t;
  _last_log_num_type last_log_num;
  using _time_utc_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _time_utc_type time_utc;
  using _size_type =
    uint32_t;
  _size_type size;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint16_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__num_logs(
    const uint16_t & _arg)
  {
    this->num_logs = _arg;
    return *this;
  }
  Type & set__last_log_num(
    const uint16_t & _arg)
  {
    this->last_log_num = _arg;
    return *this;
  }
  Type & set__time_utc(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->time_utc = _arg;
    return *this;
  }
  Type & set__size(
    const uint32_t & _arg)
  {
    this->size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::LogEntry_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::LogEntry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::LogEntry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::LogEntry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__LogEntry
    std::shared_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__LogEntry
    std::shared_ptr<mavros_msgs::msg::LogEntry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LogEntry_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->num_logs != other.num_logs) {
      return false;
    }
    if (this->last_log_num != other.last_log_num) {
      return false;
    }
    if (this->time_utc != other.time_utc) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    return true;
  }
  bool operator!=(const LogEntry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LogEntry_

// alias to use template instance with default allocator
using LogEntry =
  mavros_msgs::msg::LogEntry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__STRUCT_HPP_
