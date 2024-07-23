// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/HilActuatorControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__HilActuatorControls __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__HilActuatorControls __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HilActuatorControls_
{
  using Type = HilActuatorControls_<ContainerAllocator>;

  explicit HilActuatorControls_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 16>::iterator, float>(this->controls.begin(), this->controls.end(), 0.0f);
      this->mode = 0;
      this->flags = 0ull;
    }
  }

  explicit HilActuatorControls_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    controls(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 16>::iterator, float>(this->controls.begin(), this->controls.end(), 0.0f);
      this->mode = 0;
      this->flags = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _controls_type =
    std::array<float, 16>;
  _controls_type controls;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _flags_type =
    uint64_t;
  _flags_type flags;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__controls(
    const std::array<float, 16> & _arg)
  {
    this->controls = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__flags(
    const uint64_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::HilActuatorControls_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::HilActuatorControls_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilActuatorControls_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilActuatorControls_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__HilActuatorControls
    std::shared_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__HilActuatorControls
    std::shared_ptr<mavros_msgs::msg::HilActuatorControls_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HilActuatorControls_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->controls != other.controls) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const HilActuatorControls_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HilActuatorControls_

// alias to use template instance with default allocator
using HilActuatorControls =
  mavros_msgs::msg::HilActuatorControls_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__STRUCT_HPP_
