// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_HPP_

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
// Member 'point_1'
// Member 'point_2'
// Member 'point_3'
// Member 'point_4'
// Member 'point_5'
#include "mavros_msgs/msg/detail/position_target__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__Trajectory __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Trajectory __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Trajectory_
{
  using Type = Trajectory_<ContainerAllocator>;

  explicit Trajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    point_1(_init),
    point_2(_init),
    point_3(_init),
    point_4(_init),
    point_5(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      std::fill<typename std::array<uint8_t, 5>::iterator, uint8_t>(this->point_valid.begin(), this->point_valid.end(), 0);
      std::fill<typename std::array<uint16_t, 5>::iterator, uint16_t>(this->command.begin(), this->command.end(), 0);
      std::fill<typename std::array<float, 5>::iterator, float>(this->time_horizon.begin(), this->time_horizon.end(), 0.0f);
    }
  }

  explicit Trajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    point_1(_alloc, _init),
    point_2(_alloc, _init),
    point_3(_alloc, _init),
    point_4(_alloc, _init),
    point_5(_alloc, _init),
    point_valid(_alloc),
    command(_alloc),
    time_horizon(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      std::fill<typename std::array<uint8_t, 5>::iterator, uint8_t>(this->point_valid.begin(), this->point_valid.end(), 0);
      std::fill<typename std::array<uint16_t, 5>::iterator, uint16_t>(this->command.begin(), this->command.end(), 0);
      std::fill<typename std::array<float, 5>::iterator, float>(this->time_horizon.begin(), this->time_horizon.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _type_type =
    uint8_t;
  _type_type type;
  using _point_1_type =
    mavros_msgs::msg::PositionTarget_<ContainerAllocator>;
  _point_1_type point_1;
  using _point_2_type =
    mavros_msgs::msg::PositionTarget_<ContainerAllocator>;
  _point_2_type point_2;
  using _point_3_type =
    mavros_msgs::msg::PositionTarget_<ContainerAllocator>;
  _point_3_type point_3;
  using _point_4_type =
    mavros_msgs::msg::PositionTarget_<ContainerAllocator>;
  _point_4_type point_4;
  using _point_5_type =
    mavros_msgs::msg::PositionTarget_<ContainerAllocator>;
  _point_5_type point_5;
  using _point_valid_type =
    std::array<uint8_t, 5>;
  _point_valid_type point_valid;
  using _command_type =
    std::array<uint16_t, 5>;
  _command_type command;
  using _time_horizon_type =
    std::array<float, 5>;
  _time_horizon_type time_horizon;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__point_1(
    const mavros_msgs::msg::PositionTarget_<ContainerAllocator> & _arg)
  {
    this->point_1 = _arg;
    return *this;
  }
  Type & set__point_2(
    const mavros_msgs::msg::PositionTarget_<ContainerAllocator> & _arg)
  {
    this->point_2 = _arg;
    return *this;
  }
  Type & set__point_3(
    const mavros_msgs::msg::PositionTarget_<ContainerAllocator> & _arg)
  {
    this->point_3 = _arg;
    return *this;
  }
  Type & set__point_4(
    const mavros_msgs::msg::PositionTarget_<ContainerAllocator> & _arg)
  {
    this->point_4 = _arg;
    return *this;
  }
  Type & set__point_5(
    const mavros_msgs::msg::PositionTarget_<ContainerAllocator> & _arg)
  {
    this->point_5 = _arg;
    return *this;
  }
  Type & set__point_valid(
    const std::array<uint8_t, 5> & _arg)
  {
    this->point_valid = _arg;
    return *this;
  }
  Type & set__command(
    const std::array<uint16_t, 5> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__time_horizon(
    const std::array<float, 5> & _arg)
  {
    this->time_horizon = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS =
    0u;
  static constexpr uint8_t MAV_TRAJECTORY_REPRESENTATION_BEZIER =
    1u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Trajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Trajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Trajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Trajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Trajectory
    std::shared_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Trajectory
    std::shared_ptr<mavros_msgs::msg::Trajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Trajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->point_1 != other.point_1) {
      return false;
    }
    if (this->point_2 != other.point_2) {
      return false;
    }
    if (this->point_3 != other.point_3) {
      return false;
    }
    if (this->point_4 != other.point_4) {
      return false;
    }
    if (this->point_5 != other.point_5) {
      return false;
    }
    if (this->point_valid != other.point_valid) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->time_horizon != other.time_horizon) {
      return false;
    }
    return true;
  }
  bool operator!=(const Trajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Trajectory_

// alias to use template instance with default allocator
using Trajectory =
  mavros_msgs::msg::Trajectory_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Trajectory_<ContainerAllocator>::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Trajectory_<ContainerAllocator>::MAV_TRAJECTORY_REPRESENTATION_BEZIER;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_HPP_
