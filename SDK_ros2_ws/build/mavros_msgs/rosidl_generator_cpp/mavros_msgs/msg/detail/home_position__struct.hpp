// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/HomePosition.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__STRUCT_HPP_

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
// Member 'geo'
#include "geographic_msgs/msg/detail/geo_point__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'approach'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__HomePosition __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__HomePosition __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HomePosition_
{
  using Type = HomePosition_<ContainerAllocator>;

  explicit HomePosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    geo(_init),
    position(_init),
    orientation(_init),
    approach(_init)
  {
    (void)_init;
  }

  explicit HomePosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    geo(_alloc, _init),
    position(_alloc, _init),
    orientation(_alloc, _init),
    approach(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _geo_type =
    geographic_msgs::msg::GeoPoint_<ContainerAllocator>;
  _geo_type geo;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _approach_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _approach_type approach;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__geo(
    const geographic_msgs::msg::GeoPoint_<ContainerAllocator> & _arg)
  {
    this->geo = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__approach(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->approach = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::HomePosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::HomePosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HomePosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HomePosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__HomePosition
    std::shared_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__HomePosition
    std::shared_ptr<mavros_msgs::msg::HomePosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HomePosition_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->geo != other.geo) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->approach != other.approach) {
      return false;
    }
    return true;
  }
  bool operator!=(const HomePosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HomePosition_

// alias to use template instance with default allocator
using HomePosition =
  mavros_msgs::msg::HomePosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__STRUCT_HPP_
