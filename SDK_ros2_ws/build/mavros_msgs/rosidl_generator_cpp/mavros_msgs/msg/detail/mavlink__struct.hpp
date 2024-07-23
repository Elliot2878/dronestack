// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Mavlink.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAVLINK__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAVLINK__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__Mavlink __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Mavlink __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Mavlink_
{
  using Type = Mavlink_<ContainerAllocator>;

  explicit Mavlink_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->framing_status = 0;
      this->magic = 0;
      this->len = 0;
      this->incompat_flags = 0;
      this->compat_flags = 0;
      this->seq = 0;
      this->sysid = 0;
      this->compid = 0;
      this->msgid = 0ul;
      this->checksum = 0;
    }
  }

  explicit Mavlink_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->framing_status = 0;
      this->magic = 0;
      this->len = 0;
      this->incompat_flags = 0;
      this->compat_flags = 0;
      this->seq = 0;
      this->sysid = 0;
      this->compid = 0;
      this->msgid = 0ul;
      this->checksum = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _framing_status_type =
    uint8_t;
  _framing_status_type framing_status;
  using _magic_type =
    uint8_t;
  _magic_type magic;
  using _len_type =
    uint8_t;
  _len_type len;
  using _incompat_flags_type =
    uint8_t;
  _incompat_flags_type incompat_flags;
  using _compat_flags_type =
    uint8_t;
  _compat_flags_type compat_flags;
  using _seq_type =
    uint8_t;
  _seq_type seq;
  using _sysid_type =
    uint8_t;
  _sysid_type sysid;
  using _compid_type =
    uint8_t;
  _compid_type compid;
  using _msgid_type =
    uint32_t;
  _msgid_type msgid;
  using _checksum_type =
    uint16_t;
  _checksum_type checksum;
  using _payload64_type =
    rosidl_runtime_cpp::BoundedVector<uint64_t, 33, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint64_t>>;
  _payload64_type payload64;
  using _signature_type =
    rosidl_runtime_cpp::BoundedVector<uint8_t, 13, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _signature_type signature;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__framing_status(
    const uint8_t & _arg)
  {
    this->framing_status = _arg;
    return *this;
  }
  Type & set__magic(
    const uint8_t & _arg)
  {
    this->magic = _arg;
    return *this;
  }
  Type & set__len(
    const uint8_t & _arg)
  {
    this->len = _arg;
    return *this;
  }
  Type & set__incompat_flags(
    const uint8_t & _arg)
  {
    this->incompat_flags = _arg;
    return *this;
  }
  Type & set__compat_flags(
    const uint8_t & _arg)
  {
    this->compat_flags = _arg;
    return *this;
  }
  Type & set__seq(
    const uint8_t & _arg)
  {
    this->seq = _arg;
    return *this;
  }
  Type & set__sysid(
    const uint8_t & _arg)
  {
    this->sysid = _arg;
    return *this;
  }
  Type & set__compid(
    const uint8_t & _arg)
  {
    this->compid = _arg;
    return *this;
  }
  Type & set__msgid(
    const uint32_t & _arg)
  {
    this->msgid = _arg;
    return *this;
  }
  Type & set__checksum(
    const uint16_t & _arg)
  {
    this->checksum = _arg;
    return *this;
  }
  Type & set__payload64(
    const rosidl_runtime_cpp::BoundedVector<uint64_t, 33, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint64_t>> & _arg)
  {
    this->payload64 = _arg;
    return *this;
  }
  Type & set__signature(
    const rosidl_runtime_cpp::BoundedVector<uint8_t, 13, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->signature = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FRAMING_OK =
    1u;
  static constexpr uint8_t FRAMING_BAD_CRC =
    2u;
  static constexpr uint8_t FRAMING_BAD_SIGNATURE =
    3u;
  static constexpr uint8_t MAVLINK_V10 =
    254u;
  static constexpr uint8_t MAVLINK_V20 =
    253u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Mavlink_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Mavlink_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Mavlink_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Mavlink_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Mavlink
    std::shared_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Mavlink
    std::shared_ptr<mavros_msgs::msg::Mavlink_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Mavlink_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->framing_status != other.framing_status) {
      return false;
    }
    if (this->magic != other.magic) {
      return false;
    }
    if (this->len != other.len) {
      return false;
    }
    if (this->incompat_flags != other.incompat_flags) {
      return false;
    }
    if (this->compat_flags != other.compat_flags) {
      return false;
    }
    if (this->seq != other.seq) {
      return false;
    }
    if (this->sysid != other.sysid) {
      return false;
    }
    if (this->compid != other.compid) {
      return false;
    }
    if (this->msgid != other.msgid) {
      return false;
    }
    if (this->checksum != other.checksum) {
      return false;
    }
    if (this->payload64 != other.payload64) {
      return false;
    }
    if (this->signature != other.signature) {
      return false;
    }
    return true;
  }
  bool operator!=(const Mavlink_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Mavlink_

// alias to use template instance with default allocator
using Mavlink =
  mavros_msgs::msg::Mavlink_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mavlink_<ContainerAllocator>::FRAMING_OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mavlink_<ContainerAllocator>::FRAMING_BAD_CRC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mavlink_<ContainerAllocator>::FRAMING_BAD_SIGNATURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mavlink_<ContainerAllocator>::MAVLINK_V10;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mavlink_<ContainerAllocator>::MAVLINK_V20;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MAVLINK__STRUCT_HPP_
