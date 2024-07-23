// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/GimbalDeviceSetAttitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'q'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__GimbalDeviceSetAttitude __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__GimbalDeviceSetAttitude __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalDeviceSetAttitude_
{
  using Type = GimbalDeviceSetAttitude_<ContainerAllocator>;

  explicit GimbalDeviceSetAttitude_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : q(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_system = 0;
      this->target_component = 0;
      this->flags = 0;
      this->angular_velocity_x = 0.0f;
      this->angular_velocity_y = 0.0f;
      this->angular_velocity_z = 0.0f;
    }
  }

  explicit GimbalDeviceSetAttitude_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : q(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_system = 0;
      this->target_component = 0;
      this->flags = 0;
      this->angular_velocity_x = 0.0f;
      this->angular_velocity_y = 0.0f;
      this->angular_velocity_z = 0.0f;
    }
  }

  // field types and members
  using _target_system_type =
    uint8_t;
  _target_system_type target_system;
  using _target_component_type =
    uint8_t;
  _target_component_type target_component;
  using _flags_type =
    uint16_t;
  _flags_type flags;
  using _q_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _q_type q;
  using _angular_velocity_x_type =
    float;
  _angular_velocity_x_type angular_velocity_x;
  using _angular_velocity_y_type =
    float;
  _angular_velocity_y_type angular_velocity_y;
  using _angular_velocity_z_type =
    float;
  _angular_velocity_z_type angular_velocity_z;

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
  Type & set__flags(
    const uint16_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__q(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__angular_velocity_x(
    const float & _arg)
  {
    this->angular_velocity_x = _arg;
    return *this;
  }
  Type & set__angular_velocity_y(
    const float & _arg)
  {
    this->angular_velocity_y = _arg;
    return *this;
  }
  Type & set__angular_velocity_z(
    const float & _arg)
  {
    this->angular_velocity_z = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t FLAGS_RETRACT =
    1u;
  static constexpr uint16_t FLAGS_NEUTRAL =
    2u;
  static constexpr uint16_t FLAGS_ROLL_LOCK =
    4u;
  static constexpr uint16_t FLAGS_PITCH_LOCK =
    8u;
  static constexpr uint16_t FLAGS_YAW_LOCK =
    16u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__GimbalDeviceSetAttitude
    std::shared_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__GimbalDeviceSetAttitude
    std::shared_ptr<mavros_msgs::msg::GimbalDeviceSetAttitude_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalDeviceSetAttitude_ & other) const
  {
    if (this->target_system != other.target_system) {
      return false;
    }
    if (this->target_component != other.target_component) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->angular_velocity_x != other.angular_velocity_x) {
      return false;
    }
    if (this->angular_velocity_y != other.angular_velocity_y) {
      return false;
    }
    if (this->angular_velocity_z != other.angular_velocity_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalDeviceSetAttitude_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalDeviceSetAttitude_

// alias to use template instance with default allocator
using GimbalDeviceSetAttitude =
  mavros_msgs::msg::GimbalDeviceSetAttitude_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t GimbalDeviceSetAttitude_<ContainerAllocator>::FLAGS_RETRACT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t GimbalDeviceSetAttitude_<ContainerAllocator>::FLAGS_NEUTRAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t GimbalDeviceSetAttitude_<ContainerAllocator>::FLAGS_ROLL_LOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t GimbalDeviceSetAttitude_<ContainerAllocator>::FLAGS_PITCH_LOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t GimbalDeviceSetAttitude_<ContainerAllocator>::FLAGS_YAW_LOCK;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__STRUCT_HPP_
