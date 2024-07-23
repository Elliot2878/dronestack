// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Param.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PARAM__STRUCT_HPP_

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
// Member 'value'
#include "mavros_msgs/msg/detail/param_value__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__Param __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Param __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Param_
{
  using Type = Param_<ContainerAllocator>;

  explicit Param_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    value(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->param_id = "";
      this->param_index = 0;
      this->param_count = 0;
    }
  }

  explicit Param_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    param_id(_alloc),
    value(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->param_id = "";
      this->param_index = 0;
      this->param_count = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _param_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _param_id_type param_id;
  using _value_type =
    mavros_msgs::msg::ParamValue_<ContainerAllocator>;
  _value_type value;
  using _param_index_type =
    uint16_t;
  _param_index_type param_index;
  using _param_count_type =
    uint16_t;
  _param_count_type param_count;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__param_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->param_id = _arg;
    return *this;
  }
  Type & set__value(
    const mavros_msgs::msg::ParamValue_<ContainerAllocator> & _arg)
  {
    this->value = _arg;
    return *this;
  }
  Type & set__param_index(
    const uint16_t & _arg)
  {
    this->param_index = _arg;
    return *this;
  }
  Type & set__param_count(
    const uint16_t & _arg)
  {
    this->param_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Param_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Param_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Param_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Param_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Param_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Param_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Param_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Param_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Param_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Param_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Param
    std::shared_ptr<mavros_msgs::msg::Param_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Param
    std::shared_ptr<mavros_msgs::msg::Param_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Param_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->param_id != other.param_id) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    if (this->param_index != other.param_index) {
      return false;
    }
    if (this->param_count != other.param_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const Param_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Param_

// alias to use template instance with default allocator
using Param =
  mavros_msgs::msg::Param_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM__STRUCT_HPP_
