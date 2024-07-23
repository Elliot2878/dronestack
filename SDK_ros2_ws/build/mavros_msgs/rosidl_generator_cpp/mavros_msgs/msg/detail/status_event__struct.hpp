// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/StatusEvent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__StatusEvent __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__StatusEvent __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StatusEvent_
{
  using Type = StatusEvent_<ContainerAllocator>;

  explicit StatusEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->severity = 0;
      this->px4_id = 0ul;
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->arguments.begin(), this->arguments.end(), 0);
      this->sequence = 0;
    }
  }

  explicit StatusEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    arguments(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->severity = 0;
      this->px4_id = 0ul;
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->arguments.begin(), this->arguments.end(), 0);
      this->sequence = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _severity_type =
    uint8_t;
  _severity_type severity;
  using _px4_id_type =
    uint32_t;
  _px4_id_type px4_id;
  using _arguments_type =
    std::array<uint8_t, 40>;
  _arguments_type arguments;
  using _sequence_type =
    uint16_t;
  _sequence_type sequence;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__severity(
    const uint8_t & _arg)
  {
    this->severity = _arg;
    return *this;
  }
  Type & set__px4_id(
    const uint32_t & _arg)
  {
    this->px4_id = _arg;
    return *this;
  }
  Type & set__arguments(
    const std::array<uint8_t, 40> & _arg)
  {
    this->arguments = _arg;
    return *this;
  }
  Type & set__sequence(
    const uint16_t & _arg)
  {
    this->sequence = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t EMERGENCY =
    0u;
  static constexpr uint8_t ALERT =
    1u;
  static constexpr uint8_t CRITICAL =
    2u;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr uint8_t ERROR =
    3u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
  static constexpr uint8_t WARNING =
    4u;
  static constexpr uint8_t NOTICE =
    5u;
  static constexpr uint8_t INFO =
    6u;
  static constexpr uint8_t DEBUG =
    7u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::StatusEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::StatusEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::StatusEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::StatusEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__StatusEvent
    std::shared_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__StatusEvent
    std::shared_ptr<mavros_msgs::msg::StatusEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StatusEvent_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->severity != other.severity) {
      return false;
    }
    if (this->px4_id != other.px4_id) {
      return false;
    }
    if (this->arguments != other.arguments) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const StatusEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StatusEvent_

// alias to use template instance with default allocator
using StatusEvent =
  mavros_msgs::msg::StatusEvent_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::EMERGENCY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::ALERT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::CRITICAL;
#endif  // __cplusplus < 201703L
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::WARNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::NOTICE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StatusEvent_<ContainerAllocator>::DEBUG;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__STRUCT_HPP_
