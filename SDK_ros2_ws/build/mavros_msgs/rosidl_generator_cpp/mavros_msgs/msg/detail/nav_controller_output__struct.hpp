// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/NavControllerOutput.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__NavControllerOutput __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__NavControllerOutput __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavControllerOutput_
{
  using Type = NavControllerOutput_<ContainerAllocator>;

  explicit NavControllerOutput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nav_roll = 0.0f;
      this->nav_pitch = 0.0f;
      this->nav_bearing = 0;
      this->target_bearing = 0;
      this->wp_dist = 0;
      this->alt_error = 0.0f;
      this->aspd_error = 0.0f;
      this->xtrack_error = 0.0f;
    }
  }

  explicit NavControllerOutput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nav_roll = 0.0f;
      this->nav_pitch = 0.0f;
      this->nav_bearing = 0;
      this->target_bearing = 0;
      this->wp_dist = 0;
      this->alt_error = 0.0f;
      this->aspd_error = 0.0f;
      this->xtrack_error = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _nav_roll_type =
    float;
  _nav_roll_type nav_roll;
  using _nav_pitch_type =
    float;
  _nav_pitch_type nav_pitch;
  using _nav_bearing_type =
    int16_t;
  _nav_bearing_type nav_bearing;
  using _target_bearing_type =
    int16_t;
  _target_bearing_type target_bearing;
  using _wp_dist_type =
    uint16_t;
  _wp_dist_type wp_dist;
  using _alt_error_type =
    float;
  _alt_error_type alt_error;
  using _aspd_error_type =
    float;
  _aspd_error_type aspd_error;
  using _xtrack_error_type =
    float;
  _xtrack_error_type xtrack_error;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__nav_roll(
    const float & _arg)
  {
    this->nav_roll = _arg;
    return *this;
  }
  Type & set__nav_pitch(
    const float & _arg)
  {
    this->nav_pitch = _arg;
    return *this;
  }
  Type & set__nav_bearing(
    const int16_t & _arg)
  {
    this->nav_bearing = _arg;
    return *this;
  }
  Type & set__target_bearing(
    const int16_t & _arg)
  {
    this->target_bearing = _arg;
    return *this;
  }
  Type & set__wp_dist(
    const uint16_t & _arg)
  {
    this->wp_dist = _arg;
    return *this;
  }
  Type & set__alt_error(
    const float & _arg)
  {
    this->alt_error = _arg;
    return *this;
  }
  Type & set__aspd_error(
    const float & _arg)
  {
    this->aspd_error = _arg;
    return *this;
  }
  Type & set__xtrack_error(
    const float & _arg)
  {
    this->xtrack_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::NavControllerOutput_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::NavControllerOutput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::NavControllerOutput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::NavControllerOutput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__NavControllerOutput
    std::shared_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__NavControllerOutput
    std::shared_ptr<mavros_msgs::msg::NavControllerOutput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavControllerOutput_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->nav_roll != other.nav_roll) {
      return false;
    }
    if (this->nav_pitch != other.nav_pitch) {
      return false;
    }
    if (this->nav_bearing != other.nav_bearing) {
      return false;
    }
    if (this->target_bearing != other.target_bearing) {
      return false;
    }
    if (this->wp_dist != other.wp_dist) {
      return false;
    }
    if (this->alt_error != other.alt_error) {
      return false;
    }
    if (this->aspd_error != other.aspd_error) {
      return false;
    }
    if (this->xtrack_error != other.xtrack_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavControllerOutput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavControllerOutput_

// alias to use template instance with default allocator
using NavControllerOutput =
  mavros_msgs::msg::NavControllerOutput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__STRUCT_HPP_
