// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/RadioStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__RadioStatus __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__RadioStatus __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadioStatus_
{
  using Type = RadioStatus_<ContainerAllocator>;

  explicit RadioStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rssi = 0;
      this->remrssi = 0;
      this->txbuf = 0;
      this->noise = 0;
      this->remnoise = 0;
      this->rxerrors = 0;
      this->fixed = 0;
      this->rssi_dbm = 0.0f;
      this->remrssi_dbm = 0.0f;
    }
  }

  explicit RadioStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rssi = 0;
      this->remrssi = 0;
      this->txbuf = 0;
      this->noise = 0;
      this->remnoise = 0;
      this->rxerrors = 0;
      this->fixed = 0;
      this->rssi_dbm = 0.0f;
      this->remrssi_dbm = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rssi_type =
    uint8_t;
  _rssi_type rssi;
  using _remrssi_type =
    uint8_t;
  _remrssi_type remrssi;
  using _txbuf_type =
    uint8_t;
  _txbuf_type txbuf;
  using _noise_type =
    uint8_t;
  _noise_type noise;
  using _remnoise_type =
    uint8_t;
  _remnoise_type remnoise;
  using _rxerrors_type =
    uint16_t;
  _rxerrors_type rxerrors;
  using _fixed_type =
    uint16_t;
  _fixed_type fixed;
  using _rssi_dbm_type =
    float;
  _rssi_dbm_type rssi_dbm;
  using _remrssi_dbm_type =
    float;
  _remrssi_dbm_type remrssi_dbm;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rssi(
    const uint8_t & _arg)
  {
    this->rssi = _arg;
    return *this;
  }
  Type & set__remrssi(
    const uint8_t & _arg)
  {
    this->remrssi = _arg;
    return *this;
  }
  Type & set__txbuf(
    const uint8_t & _arg)
  {
    this->txbuf = _arg;
    return *this;
  }
  Type & set__noise(
    const uint8_t & _arg)
  {
    this->noise = _arg;
    return *this;
  }
  Type & set__remnoise(
    const uint8_t & _arg)
  {
    this->remnoise = _arg;
    return *this;
  }
  Type & set__rxerrors(
    const uint16_t & _arg)
  {
    this->rxerrors = _arg;
    return *this;
  }
  Type & set__fixed(
    const uint16_t & _arg)
  {
    this->fixed = _arg;
    return *this;
  }
  Type & set__rssi_dbm(
    const float & _arg)
  {
    this->rssi_dbm = _arg;
    return *this;
  }
  Type & set__remrssi_dbm(
    const float & _arg)
  {
    this->remrssi_dbm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::RadioStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::RadioStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::RadioStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::RadioStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__RadioStatus
    std::shared_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__RadioStatus
    std::shared_ptr<mavros_msgs::msg::RadioStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadioStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    if (this->remrssi != other.remrssi) {
      return false;
    }
    if (this->txbuf != other.txbuf) {
      return false;
    }
    if (this->noise != other.noise) {
      return false;
    }
    if (this->remnoise != other.remnoise) {
      return false;
    }
    if (this->rxerrors != other.rxerrors) {
      return false;
    }
    if (this->fixed != other.fixed) {
      return false;
    }
    if (this->rssi_dbm != other.rssi_dbm) {
      return false;
    }
    if (this->remrssi_dbm != other.remrssi_dbm) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadioStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadioStatus_

// alias to use template instance with default allocator
using RadioStatus =
  mavros_msgs::msg::RadioStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__STRUCT_HPP_
