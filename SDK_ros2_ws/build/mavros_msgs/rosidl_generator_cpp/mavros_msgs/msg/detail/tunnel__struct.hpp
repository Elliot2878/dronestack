// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Tunnel.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TUNNEL__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TUNNEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__Tunnel __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Tunnel __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Tunnel_
{
  using Type = Tunnel_<ContainerAllocator>;

  explicit Tunnel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_system = 0;
      this->target_component = 0;
      this->payload_type = 0;
      this->payload_length = 0;
      std::fill<typename std::array<uint8_t, 128>::iterator, uint8_t>(this->payload.begin(), this->payload.end(), 0);
    }
  }

  explicit Tunnel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : payload(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_system = 0;
      this->target_component = 0;
      this->payload_type = 0;
      this->payload_length = 0;
      std::fill<typename std::array<uint8_t, 128>::iterator, uint8_t>(this->payload.begin(), this->payload.end(), 0);
    }
  }

  // field types and members
  using _target_system_type =
    uint8_t;
  _target_system_type target_system;
  using _target_component_type =
    uint8_t;
  _target_component_type target_component;
  using _payload_type_type =
    uint16_t;
  _payload_type_type payload_type;
  using _payload_length_type =
    uint8_t;
  _payload_length_type payload_length;
  using _payload_type =
    std::array<uint8_t, 128>;
  _payload_type payload;

  // setters for named parameter idiom
  Type & set__target_system(
    const uint8_t & _arg)
  {
    this->target_system = _arg;
    return *this;
  }
  Type & set__target_component(
    const uint8_t & _arg)
  {
    this->target_component = _arg;
    return *this;
  }
  Type & set__payload_type(
    const uint16_t & _arg)
  {
    this->payload_type = _arg;
    return *this;
  }
  Type & set__payload_length(
    const uint8_t & _arg)
  {
    this->payload_length = _arg;
    return *this;
  }
  Type & set__payload(
    const std::array<uint8_t, 128> & _arg)
  {
    this->payload = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t PAYLOAD_TYPE_UNKNOWN =
    0u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED0 =
    200u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED1 =
    201u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED2 =
    202u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED3 =
    203u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED4 =
    204u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED5 =
    205u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED6 =
    206u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED7 =
    207u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED8 =
    208u;
  static constexpr uint16_t PAYLOAD_TYPE_STORM32_RESERVED9 =
    209u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Tunnel_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Tunnel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Tunnel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Tunnel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Tunnel
    std::shared_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Tunnel
    std::shared_ptr<mavros_msgs::msg::Tunnel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Tunnel_ & other) const
  {
    if (this->target_system != other.target_system) {
      return false;
    }
    if (this->target_component != other.target_component) {
      return false;
    }
    if (this->payload_type != other.payload_type) {
      return false;
    }
    if (this->payload_length != other.payload_length) {
      return false;
    }
    if (this->payload != other.payload) {
      return false;
    }
    return true;
  }
  bool operator!=(const Tunnel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Tunnel_

// alias to use template instance with default allocator
using Tunnel =
  mavros_msgs::msg::Tunnel_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED0;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED3;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED4;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED5;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED6;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED7;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED8;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Tunnel_<ContainerAllocator>::PAYLOAD_TYPE_STORM32_RESERVED9;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TUNNEL__STRUCT_HPP_
