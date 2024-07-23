// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/VfrHud.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VFR_HUD__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VFR_HUD__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__VfrHud __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__VfrHud __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VfrHud_
{
  using Type = VfrHud_<ContainerAllocator>;

  explicit VfrHud_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->airspeed = 0.0f;
      this->groundspeed = 0.0f;
      this->heading = 0;
      this->throttle = 0.0f;
      this->altitude = 0.0f;
      this->climb = 0.0f;
    }
  }

  explicit VfrHud_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->airspeed = 0.0f;
      this->groundspeed = 0.0f;
      this->heading = 0;
      this->throttle = 0.0f;
      this->altitude = 0.0f;
      this->climb = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _airspeed_type =
    float;
  _airspeed_type airspeed;
  using _groundspeed_type =
    float;
  _groundspeed_type groundspeed;
  using _heading_type =
    int16_t;
  _heading_type heading;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _climb_type =
    float;
  _climb_type climb;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__airspeed(
    const float & _arg)
  {
    this->airspeed = _arg;
    return *this;
  }
  Type & set__groundspeed(
    const float & _arg)
  {
    this->groundspeed = _arg;
    return *this;
  }
  Type & set__heading(
    const int16_t & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__climb(
    const float & _arg)
  {
    this->climb = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::VfrHud_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::VfrHud_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::VfrHud_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::VfrHud_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__VfrHud
    std::shared_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__VfrHud
    std::shared_ptr<mavros_msgs::msg::VfrHud_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VfrHud_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->airspeed != other.airspeed) {
      return false;
    }
    if (this->groundspeed != other.groundspeed) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->climb != other.climb) {
      return false;
    }
    return true;
  }
  bool operator!=(const VfrHud_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VfrHud_

// alias to use template instance with default allocator
using VfrHud =
  mavros_msgs::msg::VfrHud_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__VFR_HUD__STRUCT_HPP_
