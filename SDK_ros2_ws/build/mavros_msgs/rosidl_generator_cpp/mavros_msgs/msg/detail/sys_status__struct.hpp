// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/SysStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__SysStatus __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__SysStatus __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SysStatus_
{
  using Type = SysStatus_<ContainerAllocator>;

  explicit SysStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensors_present = 0ul;
      this->sensors_enabled = 0ul;
      this->sensors_health = 0ul;
      this->load = 0;
      this->voltage_battery = 0;
      this->current_battery = 0;
      this->battery_remaining = 0;
      this->drop_rate_comm = 0;
      this->errors_comm = 0;
      this->errors_count1 = 0;
      this->errors_count2 = 0;
      this->errors_count3 = 0;
      this->errors_count4 = 0;
    }
  }

  explicit SysStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensors_present = 0ul;
      this->sensors_enabled = 0ul;
      this->sensors_health = 0ul;
      this->load = 0;
      this->voltage_battery = 0;
      this->current_battery = 0;
      this->battery_remaining = 0;
      this->drop_rate_comm = 0;
      this->errors_comm = 0;
      this->errors_count1 = 0;
      this->errors_count2 = 0;
      this->errors_count3 = 0;
      this->errors_count4 = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _sensors_present_type =
    uint32_t;
  _sensors_present_type sensors_present;
  using _sensors_enabled_type =
    uint32_t;
  _sensors_enabled_type sensors_enabled;
  using _sensors_health_type =
    uint32_t;
  _sensors_health_type sensors_health;
  using _load_type =
    uint16_t;
  _load_type load;
  using _voltage_battery_type =
    uint16_t;
  _voltage_battery_type voltage_battery;
  using _current_battery_type =
    int16_t;
  _current_battery_type current_battery;
  using _battery_remaining_type =
    int8_t;
  _battery_remaining_type battery_remaining;
  using _drop_rate_comm_type =
    uint16_t;
  _drop_rate_comm_type drop_rate_comm;
  using _errors_comm_type =
    uint16_t;
  _errors_comm_type errors_comm;
  using _errors_count1_type =
    uint16_t;
  _errors_count1_type errors_count1;
  using _errors_count2_type =
    uint16_t;
  _errors_count2_type errors_count2;
  using _errors_count3_type =
    uint16_t;
  _errors_count3_type errors_count3;
  using _errors_count4_type =
    uint16_t;
  _errors_count4_type errors_count4;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__sensors_present(
    const uint32_t & _arg)
  {
    this->sensors_present = _arg;
    return *this;
  }
  Type & set__sensors_enabled(
    const uint32_t & _arg)
  {
    this->sensors_enabled = _arg;
    return *this;
  }
  Type & set__sensors_health(
    const uint32_t & _arg)
  {
    this->sensors_health = _arg;
    return *this;
  }
  Type & set__load(
    const uint16_t & _arg)
  {
    this->load = _arg;
    return *this;
  }
  Type & set__voltage_battery(
    const uint16_t & _arg)
  {
    this->voltage_battery = _arg;
    return *this;
  }
  Type & set__current_battery(
    const int16_t & _arg)
  {
    this->current_battery = _arg;
    return *this;
  }
  Type & set__battery_remaining(
    const int8_t & _arg)
  {
    this->battery_remaining = _arg;
    return *this;
  }
  Type & set__drop_rate_comm(
    const uint16_t & _arg)
  {
    this->drop_rate_comm = _arg;
    return *this;
  }
  Type & set__errors_comm(
    const uint16_t & _arg)
  {
    this->errors_comm = _arg;
    return *this;
  }
  Type & set__errors_count1(
    const uint16_t & _arg)
  {
    this->errors_count1 = _arg;
    return *this;
  }
  Type & set__errors_count2(
    const uint16_t & _arg)
  {
    this->errors_count2 = _arg;
    return *this;
  }
  Type & set__errors_count3(
    const uint16_t & _arg)
  {
    this->errors_count3 = _arg;
    return *this;
  }
  Type & set__errors_count4(
    const uint16_t & _arg)
  {
    this->errors_count4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::SysStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::SysStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::SysStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::SysStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__SysStatus
    std::shared_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__SysStatus
    std::shared_ptr<mavros_msgs::msg::SysStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SysStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->sensors_present != other.sensors_present) {
      return false;
    }
    if (this->sensors_enabled != other.sensors_enabled) {
      return false;
    }
    if (this->sensors_health != other.sensors_health) {
      return false;
    }
    if (this->load != other.load) {
      return false;
    }
    if (this->voltage_battery != other.voltage_battery) {
      return false;
    }
    if (this->current_battery != other.current_battery) {
      return false;
    }
    if (this->battery_remaining != other.battery_remaining) {
      return false;
    }
    if (this->drop_rate_comm != other.drop_rate_comm) {
      return false;
    }
    if (this->errors_comm != other.errors_comm) {
      return false;
    }
    if (this->errors_count1 != other.errors_count1) {
      return false;
    }
    if (this->errors_count2 != other.errors_count2) {
      return false;
    }
    if (this->errors_count3 != other.errors_count3) {
      return false;
    }
    if (this->errors_count4 != other.errors_count4) {
      return false;
    }
    return true;
  }
  bool operator!=(const SysStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SysStatus_

// alias to use template instance with default allocator
using SysStatus =
  mavros_msgs::msg::SysStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__STRUCT_HPP_
