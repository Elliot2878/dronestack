// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/PlayTuneV2.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__PlayTuneV2 __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__PlayTuneV2 __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlayTuneV2_
{
  using Type = PlayTuneV2_<ContainerAllocator>;

  explicit PlayTuneV2_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->format = 0;
      this->tune = "";
    }
  }

  explicit PlayTuneV2_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tune(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->format = 0;
      this->tune = "";
    }
  }

  // field types and members
  using _format_type =
    uint8_t;
  _format_type format;
  using _tune_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tune_type tune;

  // setters for named parameter idiom
  Type & set__format(
    const uint8_t & _arg)
  {
    this->format = _arg;
    return *this;
  }
  Type & set__tune(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tune = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t QBASIC1_1 =
    1u;
  static constexpr uint8_t MML_MODERN =
    2u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::PlayTuneV2_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::PlayTuneV2_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::PlayTuneV2_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::PlayTuneV2_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__PlayTuneV2
    std::shared_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__PlayTuneV2
    std::shared_ptr<mavros_msgs::msg::PlayTuneV2_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlayTuneV2_ & other) const
  {
    if (this->format != other.format) {
      return false;
    }
    if (this->tune != other.tune) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlayTuneV2_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlayTuneV2_

// alias to use template instance with default allocator
using PlayTuneV2 =
  mavros_msgs::msg::PlayTuneV2_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlayTuneV2_<ContainerAllocator>::QBASIC1_1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlayTuneV2_<ContainerAllocator>::MML_MODERN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__STRUCT_HPP_
