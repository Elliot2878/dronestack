// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/OpticalFlow.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__STRUCT_HPP_

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
// Member 'flow'
// Member 'flow_comp_m'
// Member 'flow_rate'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__OpticalFlow __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__OpticalFlow __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpticalFlow_
{
  using Type = OpticalFlow_<ContainerAllocator>;

  explicit OpticalFlow_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    flow(_init),
    flow_comp_m(_init),
    flow_rate(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quality = 0;
      this->ground_distance = 0.0f;
    }
  }

  explicit OpticalFlow_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    flow(_alloc, _init),
    flow_comp_m(_alloc, _init),
    flow_rate(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quality = 0;
      this->ground_distance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _flow_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _flow_type flow;
  using _flow_comp_m_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _flow_comp_m_type flow_comp_m;
  using _quality_type =
    uint8_t;
  _quality_type quality;
  using _ground_distance_type =
    float;
  _ground_distance_type ground_distance;
  using _flow_rate_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _flow_rate_type flow_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__flow(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->flow = _arg;
    return *this;
  }
  Type & set__flow_comp_m(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->flow_comp_m = _arg;
    return *this;
  }
  Type & set__quality(
    const uint8_t & _arg)
  {
    this->quality = _arg;
    return *this;
  }
  Type & set__ground_distance(
    const float & _arg)
  {
    this->ground_distance = _arg;
    return *this;
  }
  Type & set__flow_rate(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->flow_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::OpticalFlow_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::OpticalFlow_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpticalFlow_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpticalFlow_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__OpticalFlow
    std::shared_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__OpticalFlow
    std::shared_ptr<mavros_msgs::msg::OpticalFlow_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpticalFlow_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->flow != other.flow) {
      return false;
    }
    if (this->flow_comp_m != other.flow_comp_m) {
      return false;
    }
    if (this->quality != other.quality) {
      return false;
    }
    if (this->ground_distance != other.ground_distance) {
      return false;
    }
    if (this->flow_rate != other.flow_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpticalFlow_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpticalFlow_

// alias to use template instance with default allocator
using OpticalFlow =
  mavros_msgs::msg::OpticalFlow_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__STRUCT_HPP_
