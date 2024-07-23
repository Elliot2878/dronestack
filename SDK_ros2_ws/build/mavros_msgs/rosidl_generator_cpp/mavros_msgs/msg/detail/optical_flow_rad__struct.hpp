// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/OpticalFlowRad.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__OpticalFlowRad __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__OpticalFlowRad __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpticalFlowRad_
{
  using Type = OpticalFlowRad_<ContainerAllocator>;

  explicit OpticalFlowRad_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integration_time_us = 0ul;
      this->integrated_x = 0.0f;
      this->integrated_y = 0.0f;
      this->integrated_xgyro = 0.0f;
      this->integrated_ygyro = 0.0f;
      this->integrated_zgyro = 0.0f;
      this->temperature = 0;
      this->quality = 0;
      this->time_delta_distance_us = 0ul;
      this->distance = 0.0f;
    }
  }

  explicit OpticalFlowRad_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integration_time_us = 0ul;
      this->integrated_x = 0.0f;
      this->integrated_y = 0.0f;
      this->integrated_xgyro = 0.0f;
      this->integrated_ygyro = 0.0f;
      this->integrated_zgyro = 0.0f;
      this->temperature = 0;
      this->quality = 0;
      this->time_delta_distance_us = 0ul;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _integration_time_us_type =
    uint32_t;
  _integration_time_us_type integration_time_us;
  using _integrated_x_type =
    float;
  _integrated_x_type integrated_x;
  using _integrated_y_type =
    float;
  _integrated_y_type integrated_y;
  using _integrated_xgyro_type =
    float;
  _integrated_xgyro_type integrated_xgyro;
  using _integrated_ygyro_type =
    float;
  _integrated_ygyro_type integrated_ygyro;
  using _integrated_zgyro_type =
    float;
  _integrated_zgyro_type integrated_zgyro;
  using _temperature_type =
    int16_t;
  _temperature_type temperature;
  using _quality_type =
    uint8_t;
  _quality_type quality;
  using _time_delta_distance_us_type =
    uint32_t;
  _time_delta_distance_us_type time_delta_distance_us;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__integration_time_us(
    const uint32_t & _arg)
  {
    this->integration_time_us = _arg;
    return *this;
  }
  Type & set__integrated_x(
    const float & _arg)
  {
    this->integrated_x = _arg;
    return *this;
  }
  Type & set__integrated_y(
    const float & _arg)
  {
    this->integrated_y = _arg;
    return *this;
  }
  Type & set__integrated_xgyro(
    const float & _arg)
  {
    this->integrated_xgyro = _arg;
    return *this;
  }
  Type & set__integrated_ygyro(
    const float & _arg)
  {
    this->integrated_ygyro = _arg;
    return *this;
  }
  Type & set__integrated_zgyro(
    const float & _arg)
  {
    this->integrated_zgyro = _arg;
    return *this;
  }
  Type & set__temperature(
    const int16_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__quality(
    const uint8_t & _arg)
  {
    this->quality = _arg;
    return *this;
  }
  Type & set__time_delta_distance_us(
    const uint32_t & _arg)
  {
    this->time_delta_distance_us = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__OpticalFlowRad
    std::shared_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__OpticalFlowRad
    std::shared_ptr<mavros_msgs::msg::OpticalFlowRad_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpticalFlowRad_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->integration_time_us != other.integration_time_us) {
      return false;
    }
    if (this->integrated_x != other.integrated_x) {
      return false;
    }
    if (this->integrated_y != other.integrated_y) {
      return false;
    }
    if (this->integrated_xgyro != other.integrated_xgyro) {
      return false;
    }
    if (this->integrated_ygyro != other.integrated_ygyro) {
      return false;
    }
    if (this->integrated_zgyro != other.integrated_zgyro) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->quality != other.quality) {
      return false;
    }
    if (this->time_delta_distance_us != other.time_delta_distance_us) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpticalFlowRad_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpticalFlowRad_

// alias to use template instance with default allocator
using OpticalFlowRad =
  mavros_msgs::msg::OpticalFlowRad_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__STRUCT_HPP_
