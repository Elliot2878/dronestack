// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/GimbalManagerStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__GimbalManagerStatus __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__GimbalManagerStatus __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerStatus_
{
  using Type = GimbalManagerStatus_<ContainerAllocator>;

  explicit GimbalManagerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flags = 0ul;
      this->gimbal_device_id = 0;
      this->sysid_primary = 0;
      this->compid_primary = 0;
      this->sysid_secondary = 0;
      this->compid_secondary = 0;
    }
  }

  explicit GimbalManagerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flags = 0ul;
      this->gimbal_device_id = 0;
      this->sysid_primary = 0;
      this->compid_primary = 0;
      this->sysid_secondary = 0;
      this->compid_secondary = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _flags_type =
    uint32_t;
  _flags_type flags;
  using _gimbal_device_id_type =
    uint8_t;
  _gimbal_device_id_type gimbal_device_id;
  using _sysid_primary_type =
    uint8_t;
  _sysid_primary_type sysid_primary;
  using _compid_primary_type =
    uint8_t;
  _compid_primary_type compid_primary;
  using _sysid_secondary_type =
    uint8_t;
  _sysid_secondary_type sysid_secondary;
  using _compid_secondary_type =
    uint8_t;
  _compid_secondary_type compid_secondary;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__flags(
    const uint32_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__gimbal_device_id(
    const uint8_t & _arg)
  {
    this->gimbal_device_id = _arg;
    return *this;
  }
  Type & set__sysid_primary(
    const uint8_t & _arg)
  {
    this->sysid_primary = _arg;
    return *this;
  }
  Type & set__compid_primary(
    const uint8_t & _arg)
  {
    this->compid_primary = _arg;
    return *this;
  }
  Type & set__sysid_secondary(
    const uint8_t & _arg)
  {
    this->sysid_secondary = _arg;
    return *this;
  }
  Type & set__compid_secondary(
    const uint8_t & _arg)
  {
    this->compid_secondary = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_RETRACT =
    1u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_NEUTRAL =
    2u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_ROLL_LOCK =
    4u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_PITCH_LOCK =
    8u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_YAW_LOCK =
    16u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__GimbalManagerStatus
    std::shared_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__GimbalManagerStatus
    std::shared_ptr<mavros_msgs::msg::GimbalManagerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    if (this->gimbal_device_id != other.gimbal_device_id) {
      return false;
    }
    if (this->sysid_primary != other.sysid_primary) {
      return false;
    }
    if (this->compid_primary != other.compid_primary) {
      return false;
    }
    if (this->sysid_secondary != other.sysid_secondary) {
      return false;
    }
    if (this->compid_secondary != other.compid_secondary) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerStatus_

// alias to use template instance with default allocator
using GimbalManagerStatus =
  mavros_msgs::msg::GimbalManagerStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerStatus_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_RETRACT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerStatus_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_NEUTRAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerStatus_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_ROLL_LOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerStatus_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_PITCH_LOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerStatus_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_YAW_LOCK;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__STRUCT_HPP_
