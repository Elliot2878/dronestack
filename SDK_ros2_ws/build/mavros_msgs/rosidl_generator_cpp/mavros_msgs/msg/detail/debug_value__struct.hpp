// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/DebugValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__DebugValue __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__DebugValue __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DebugValue_
{
  using Type = DebugValue_<ContainerAllocator>;

  explicit DebugValue_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->array_id = 0l;
      this->name = "";
      this->value_float = 0.0f;
      this->value_int = 0l;
      this->type = 0;
    }
  }

  explicit DebugValue_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->array_id = 0l;
      this->name = "";
      this->value_float = 0.0f;
      this->value_int = 0l;
      this->type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _index_type =
    int32_t;
  _index_type index;
  using _array_id_type =
    int32_t;
  _array_id_type array_id;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _value_float_type =
    float;
  _value_float_type value_float;
  using _value_int_type =
    int32_t;
  _value_int_type value_int;
  using _data_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _data_type data;
  using _type_type =
    uint8_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__index(
    const int32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__array_id(
    const int32_t & _arg)
  {
    this->array_id = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__value_float(
    const float & _arg)
  {
    this->value_float = _arg;
    return *this;
  }
  Type & set__value_int(
    const int32_t & _arg)
  {
    this->value_int = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TYPE_DEBUG =
    0u;
  static constexpr uint8_t TYPE_DEBUG_VECT =
    1u;
  static constexpr uint8_t TYPE_DEBUG_FLOAT_ARRAY =
    2u;
  static constexpr uint8_t TYPE_NAMED_VALUE_FLOAT =
    3u;
  static constexpr uint8_t TYPE_NAMED_VALUE_INT =
    4u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::DebugValue_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::DebugValue_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::DebugValue_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::DebugValue_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__DebugValue
    std::shared_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__DebugValue
    std::shared_ptr<mavros_msgs::msg::DebugValue_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DebugValue_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->array_id != other.array_id) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->value_float != other.value_float) {
      return false;
    }
    if (this->value_int != other.value_int) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const DebugValue_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DebugValue_

// alias to use template instance with default allocator
using DebugValue =
  mavros_msgs::msg::DebugValue_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DebugValue_<ContainerAllocator>::TYPE_DEBUG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DebugValue_<ContainerAllocator>::TYPE_DEBUG_VECT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DebugValue_<ContainerAllocator>::TYPE_DEBUG_FLOAT_ARRAY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DebugValue_<ContainerAllocator>::TYPE_NAMED_VALUE_FLOAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DebugValue_<ContainerAllocator>::TYPE_NAMED_VALUE_INT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__STRUCT_HPP_
