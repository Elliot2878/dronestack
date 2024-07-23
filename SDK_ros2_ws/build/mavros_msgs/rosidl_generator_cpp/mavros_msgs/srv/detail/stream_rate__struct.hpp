// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/StreamRate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__StreamRate_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__StreamRate_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StreamRate_Request_
{
  using Type = StreamRate_Request_<ContainerAllocator>;

  explicit StreamRate_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stream_id = 0;
      this->message_rate = 0;
      this->on_off = false;
    }
  }

  explicit StreamRate_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stream_id = 0;
      this->message_rate = 0;
      this->on_off = false;
    }
  }

  // field types and members
  using _stream_id_type =
    uint8_t;
  _stream_id_type stream_id;
  using _message_rate_type =
    uint16_t;
  _message_rate_type message_rate;
  using _on_off_type =
    bool;
  _on_off_type on_off;

  // setters for named parameter idiom
  Type & set__stream_id(
    const uint8_t & _arg)
  {
    this->stream_id = _arg;
    return *this;
  }
  Type & set__message_rate(
    const uint16_t & _arg)
  {
    this->message_rate = _arg;
    return *this;
  }
  Type & set__on_off(
    const bool & _arg)
  {
    this->on_off = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STREAM_ALL =
    0u;
  static constexpr uint8_t STREAM_RAW_SENSORS =
    1u;
  static constexpr uint8_t STREAM_EXTENDED_STATUS =
    2u;
  static constexpr uint8_t STREAM_RC_CHANNELS =
    3u;
  static constexpr uint8_t STREAM_RAW_CONTROLLER =
    4u;
  static constexpr uint8_t STREAM_POSITION =
    6u;
  static constexpr uint8_t STREAM_EXTRA1 =
    10u;
  static constexpr uint8_t STREAM_EXTRA2 =
    11u;
  static constexpr uint8_t STREAM_EXTRA3 =
    12u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::StreamRate_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::StreamRate_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::StreamRate_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::StreamRate_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__StreamRate_Request
    std::shared_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__StreamRate_Request
    std::shared_ptr<mavros_msgs::srv::StreamRate_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StreamRate_Request_ & other) const
  {
    if (this->stream_id != other.stream_id) {
      return false;
    }
    if (this->message_rate != other.message_rate) {
      return false;
    }
    if (this->on_off != other.on_off) {
      return false;
    }
    return true;
  }
  bool operator!=(const StreamRate_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StreamRate_Request_

// alias to use template instance with default allocator
using StreamRate_Request =
  mavros_msgs::srv::StreamRate_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_ALL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_RAW_SENSORS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_EXTENDED_STATUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_RC_CHANNELS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_RAW_CONTROLLER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_EXTRA1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_EXTRA2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t StreamRate_Request_<ContainerAllocator>::STREAM_EXTRA3;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__StreamRate_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__StreamRate_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StreamRate_Response_
{
  using Type = StreamRate_Response_<ContainerAllocator>;

  explicit StreamRate_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit StreamRate_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::StreamRate_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::StreamRate_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::StreamRate_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::StreamRate_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__StreamRate_Response
    std::shared_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__StreamRate_Response
    std::shared_ptr<mavros_msgs::srv::StreamRate_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StreamRate_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const StreamRate_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StreamRate_Response_

// alias to use template instance with default allocator
using StreamRate_Response =
  mavros_msgs::srv::StreamRate_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct StreamRate
{
  using Request = mavros_msgs::srv::StreamRate_Request;
  using Response = mavros_msgs::srv::StreamRate_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__STRUCT_HPP_
