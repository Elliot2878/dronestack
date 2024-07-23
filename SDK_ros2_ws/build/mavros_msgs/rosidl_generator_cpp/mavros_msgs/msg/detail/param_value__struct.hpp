// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ParamValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__ParamValue __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ParamValue __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ParamValue_
{
  using Type = ParamValue_<ContainerAllocator>;

  explicit ParamValue_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integer = 0ll;
      this->real = 0.0;
    }
  }

  explicit ParamValue_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integer = 0ll;
      this->real = 0.0;
    }
  }

  // field types and members
  using _integer_type =
    int64_t;
  _integer_type integer;
  using _real_type =
    double;
  _real_type real;

  // setters for named parameter idiom
  Type & set__integer(
    const int64_t & _arg)
  {
    this->integer = _arg;
    return *this;
  }
  Type & set__real(
    const double & _arg)
  {
    this->real = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ParamValue_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ParamValue_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ParamValue_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ParamValue_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ParamValue
    std::shared_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ParamValue
    std::shared_ptr<mavros_msgs::msg::ParamValue_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParamValue_ & other) const
  {
    if (this->integer != other.integer) {
      return false;
    }
    if (this->real != other.real) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParamValue_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParamValue_

// alias to use template instance with default allocator
using ParamValue =
  mavros_msgs::msg::ParamValue_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__STRUCT_HPP_
