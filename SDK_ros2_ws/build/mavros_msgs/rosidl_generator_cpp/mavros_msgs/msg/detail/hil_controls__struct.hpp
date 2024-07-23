// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/HilControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__HilControls __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__HilControls __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HilControls_
{
  using Type = HilControls_<ContainerAllocator>;

  explicit HilControls_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->roll_ailerons = 0.0f;
      this->pitch_elevator = 0.0f;
      this->yaw_rudder = 0.0f;
      this->throttle = 0.0f;
      this->aux1 = 0.0f;
      this->aux2 = 0.0f;
      this->aux3 = 0.0f;
      this->aux4 = 0.0f;
      this->mode = 0;
      this->nav_mode = 0;
    }
  }

  explicit HilControls_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->roll_ailerons = 0.0f;
      this->pitch_elevator = 0.0f;
      this->yaw_rudder = 0.0f;
      this->throttle = 0.0f;
      this->aux1 = 0.0f;
      this->aux2 = 0.0f;
      this->aux3 = 0.0f;
      this->aux4 = 0.0f;
      this->mode = 0;
      this->nav_mode = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _roll_ailerons_type =
    float;
  _roll_ailerons_type roll_ailerons;
  using _pitch_elevator_type =
    float;
  _pitch_elevator_type pitch_elevator;
  using _yaw_rudder_type =
    float;
  _yaw_rudder_type yaw_rudder;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _aux1_type =
    float;
  _aux1_type aux1;
  using _aux2_type =
    float;
  _aux2_type aux2;
  using _aux3_type =
    float;
  _aux3_type aux3;
  using _aux4_type =
    float;
  _aux4_type aux4;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _nav_mode_type =
    uint8_t;
  _nav_mode_type nav_mode;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__roll_ailerons(
    const float & _arg)
  {
    this->roll_ailerons = _arg;
    return *this;
  }
  Type & set__pitch_elevator(
    const float & _arg)
  {
    this->pitch_elevator = _arg;
    return *this;
  }
  Type & set__yaw_rudder(
    const float & _arg)
  {
    this->yaw_rudder = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__aux1(
    const float & _arg)
  {
    this->aux1 = _arg;
    return *this;
  }
  Type & set__aux2(
    const float & _arg)
  {
    this->aux2 = _arg;
    return *this;
  }
  Type & set__aux3(
    const float & _arg)
  {
    this->aux3 = _arg;
    return *this;
  }
  Type & set__aux4(
    const float & _arg)
  {
    this->aux4 = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__nav_mode(
    const uint8_t & _arg)
  {
    this->nav_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::HilControls_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::HilControls_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilControls_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilControls_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__HilControls
    std::shared_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__HilControls
    std::shared_ptr<mavros_msgs::msg::HilControls_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HilControls_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->roll_ailerons != other.roll_ailerons) {
      return false;
    }
    if (this->pitch_elevator != other.pitch_elevator) {
      return false;
    }
    if (this->yaw_rudder != other.yaw_rudder) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->aux1 != other.aux1) {
      return false;
    }
    if (this->aux2 != other.aux2) {
      return false;
    }
    if (this->aux3 != other.aux3) {
      return false;
    }
    if (this->aux4 != other.aux4) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->nav_mode != other.nav_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const HilControls_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HilControls_

// alias to use template instance with default allocator
using HilControls =
  mavros_msgs::msg::HilControls_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__STRUCT_HPP_
