// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Vibration.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VIBRATION__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VIBRATION__STRUCT_HPP_

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
// Member 'vibration'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__Vibration __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Vibration __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Vibration_
{
  using Type = Vibration_<ContainerAllocator>;

  explicit Vibration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    vibration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->clipping.begin(), this->clipping.end(), 0.0f);
    }
  }

  explicit Vibration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    vibration(_alloc, _init),
    clipping(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->clipping.begin(), this->clipping.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vibration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vibration_type vibration;
  using _clipping_type =
    std::array<float, 3>;
  _clipping_type clipping;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vibration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vibration = _arg;
    return *this;
  }
  Type & set__clipping(
    const std::array<float, 3> & _arg)
  {
    this->clipping = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Vibration_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Vibration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Vibration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Vibration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Vibration
    std::shared_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Vibration
    std::shared_ptr<mavros_msgs::msg::Vibration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Vibration_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vibration != other.vibration) {
      return false;
    }
    if (this->clipping != other.clipping) {
      return false;
    }
    return true;
  }
  bool operator!=(const Vibration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Vibration_

// alias to use template instance with default allocator
using Vibration =
  mavros_msgs::msg::Vibration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__VIBRATION__STRUCT_HPP_
