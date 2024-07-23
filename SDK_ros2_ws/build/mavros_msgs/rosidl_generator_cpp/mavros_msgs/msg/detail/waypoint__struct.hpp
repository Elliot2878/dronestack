// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__Waypoint __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Waypoint __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Waypoint_
{
  using Type = Waypoint_<ContainerAllocator>;

  explicit Waypoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame = 0;
      this->command = 0;
      this->is_current = false;
      this->autocontinue = false;
      this->param1 = 0.0f;
      this->param2 = 0.0f;
      this->param3 = 0.0f;
      this->param4 = 0.0f;
      this->x_lat = 0.0;
      this->y_long = 0.0;
      this->z_alt = 0.0;
    }
  }

  explicit Waypoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame = 0;
      this->command = 0;
      this->is_current = false;
      this->autocontinue = false;
      this->param1 = 0.0f;
      this->param2 = 0.0f;
      this->param3 = 0.0f;
      this->param4 = 0.0f;
      this->x_lat = 0.0;
      this->y_long = 0.0;
      this->z_alt = 0.0;
    }
  }

  // field types and members
  using _frame_type =
    uint8_t;
  _frame_type frame;
  using _command_type =
    uint16_t;
  _command_type command;
  using _is_current_type =
    bool;
  _is_current_type is_current;
  using _autocontinue_type =
    bool;
  _autocontinue_type autocontinue;
  using _param1_type =
    float;
  _param1_type param1;
  using _param2_type =
    float;
  _param2_type param2;
  using _param3_type =
    float;
  _param3_type param3;
  using _param4_type =
    float;
  _param4_type param4;
  using _x_lat_type =
    double;
  _x_lat_type x_lat;
  using _y_long_type =
    double;
  _y_long_type y_long;
  using _z_alt_type =
    double;
  _z_alt_type z_alt;

  // setters for named parameter idiom
  Type & set__frame(
    const uint8_t & _arg)
  {
    this->frame = _arg;
    return *this;
  }
  Type & set__command(
    const uint16_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__is_current(
    const bool & _arg)
  {
    this->is_current = _arg;
    return *this;
  }
  Type & set__autocontinue(
    const bool & _arg)
  {
    this->autocontinue = _arg;
    return *this;
  }
  Type & set__param1(
    const float & _arg)
  {
    this->param1 = _arg;
    return *this;
  }
  Type & set__param2(
    const float & _arg)
  {
    this->param2 = _arg;
    return *this;
  }
  Type & set__param3(
    const float & _arg)
  {
    this->param3 = _arg;
    return *this;
  }
  Type & set__param4(
    const float & _arg)
  {
    this->param4 = _arg;
    return *this;
  }
  Type & set__x_lat(
    const double & _arg)
  {
    this->x_lat = _arg;
    return *this;
  }
  Type & set__y_long(
    const double & _arg)
  {
    this->y_long = _arg;
    return *this;
  }
  Type & set__z_alt(
    const double & _arg)
  {
    this->z_alt = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FRAME_GLOBAL =
    0u;
  static constexpr uint8_t FRAME_LOCAL_NED =
    1u;
  static constexpr uint8_t FRAME_MISSION =
    2u;
  static constexpr uint8_t FRAME_GLOBAL_REL_ALT =
    3u;
  static constexpr uint8_t FRAME_LOCAL_ENU =
    4u;
  static constexpr uint8_t FRAME_GLOBAL_INT =
    5u;
  static constexpr uint8_t FRAME_GLOBAL_RELATIVE_ALT_INT =
    6u;
  static constexpr uint8_t FRAME_LOCAL_OFFSET_NED =
    7u;
  static constexpr uint8_t FRAME_BODY_NED =
    8u;
  static constexpr uint8_t FRAME_BODY_OFFSET_NED =
    9u;
  static constexpr uint8_t FRAME_GLOBAL_TERRAIN_ALT =
    10u;
  static constexpr uint8_t FRAME_GLOBAL_TERRAIN_ALT_INT =
    11u;
  static constexpr uint8_t FRAME_BODY_FRD =
    12u;
  static constexpr uint8_t FRAME_RESERVED_13 =
    13u;
  static constexpr uint8_t FRAME_RESERVED_14 =
    14u;
  static constexpr uint8_t FRAME_RESERVED_15 =
    15u;
  static constexpr uint8_t FRAME_RESERVED_16 =
    16u;
  static constexpr uint8_t FRAME_RESERVED_17 =
    17u;
  static constexpr uint8_t FRAME_RESERVED_18 =
    18u;
  static constexpr uint8_t FRAME_RESERVED_19 =
    19u;
  static constexpr uint8_t FRAME_LOCAL_FRD =
    20u;
  static constexpr uint8_t FRAME_LOCAL_FLU =
    21u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Waypoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Waypoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Waypoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Waypoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Waypoint
    std::shared_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Waypoint
    std::shared_ptr<mavros_msgs::msg::Waypoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Waypoint_ & other) const
  {
    if (this->frame != other.frame) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->is_current != other.is_current) {
      return false;
    }
    if (this->autocontinue != other.autocontinue) {
      return false;
    }
    if (this->param1 != other.param1) {
      return false;
    }
    if (this->param2 != other.param2) {
      return false;
    }
    if (this->param3 != other.param3) {
      return false;
    }
    if (this->param4 != other.param4) {
      return false;
    }
    if (this->x_lat != other.x_lat) {
      return false;
    }
    if (this->y_long != other.y_long) {
      return false;
    }
    if (this->z_alt != other.z_alt) {
      return false;
    }
    return true;
  }
  bool operator!=(const Waypoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Waypoint_

// alias to use template instance with default allocator
using Waypoint =
  mavros_msgs::msg::Waypoint_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_GLOBAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_LOCAL_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_MISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_GLOBAL_REL_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_LOCAL_ENU;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_GLOBAL_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_GLOBAL_RELATIVE_ALT_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_LOCAL_OFFSET_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_BODY_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_BODY_OFFSET_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_GLOBAL_TERRAIN_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_GLOBAL_TERRAIN_ALT_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_BODY_FRD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_13;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_14;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_15;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_16;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_17;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_18;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_RESERVED_19;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_LOCAL_FRD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Waypoint_<ContainerAllocator>::FRAME_LOCAL_FLU;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_HPP_
