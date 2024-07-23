// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/LandingTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__LandingTarget __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__LandingTarget __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LandingTarget_
{
  using Type = LandingTarget_<ContainerAllocator>;

  explicit LandingTarget_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_num = 0;
      this->frame = 0;
      std::fill<typename std::array<float, 2>::iterator, float>(this->angle.begin(), this->angle.end(), 0.0f);
      this->distance = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->size.begin(), this->size.end(), 0.0f);
      this->type = 0;
    }
  }

  explicit LandingTarget_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    angle(_alloc),
    size(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_num = 0;
      this->frame = 0;
      std::fill<typename std::array<float, 2>::iterator, float>(this->angle.begin(), this->angle.end(), 0.0f);
      this->distance = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->size.begin(), this->size.end(), 0.0f);
      this->type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _target_num_type =
    uint8_t;
  _target_num_type target_num;
  using _frame_type =
    uint8_t;
  _frame_type frame;
  using _angle_type =
    std::array<float, 2>;
  _angle_type angle;
  using _distance_type =
    float;
  _distance_type distance;
  using _size_type =
    std::array<float, 2>;
  _size_type size;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
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
  Type & set__target_num(
    const uint8_t & _arg)
  {
    this->target_num = _arg;
    return *this;
  }
  Type & set__frame(
    const uint8_t & _arg)
  {
    this->frame = _arg;
    return *this;
  }
  Type & set__angle(
    const std::array<float, 2> & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__size(
    const std::array<float, 2> & _arg)
  {
    this->size = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GLOBAL =
    0u;
  static constexpr uint8_t LOCAL_NED =
    2u;
  static constexpr uint8_t MISSION =
    3u;
  static constexpr uint8_t GLOBAL_RELATIVE_ALT =
    4u;
  static constexpr uint8_t LOCAL_ENU =
    5u;
  static constexpr uint8_t GLOBAL_INT =
    6u;
  static constexpr uint8_t GLOBAL_RELATIVE_ALT_INT =
    7u;
  static constexpr uint8_t LOCAL_OFFSET_NED =
    8u;
  static constexpr uint8_t BODY_NED =
    9u;
  static constexpr uint8_t BODY_OFFSET_NED =
    10u;
  static constexpr uint8_t GLOBAL_TERRAIN_ALT =
    11u;
  static constexpr uint8_t GLOBAL_TERRAIN_ALT_INT =
    12u;
  static constexpr uint8_t LIGHT_BEACON =
    0u;
  static constexpr uint8_t RADIO_BEACON =
    1u;
  static constexpr uint8_t VISION_FIDUCIAL =
    2u;
  static constexpr uint8_t VISION_OTHER =
    3u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::LandingTarget_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::LandingTarget_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::LandingTarget_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::LandingTarget_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__LandingTarget
    std::shared_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__LandingTarget
    std::shared_ptr<mavros_msgs::msg::LandingTarget_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LandingTarget_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->target_num != other.target_num) {
      return false;
    }
    if (this->frame != other.frame) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const LandingTarget_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LandingTarget_

// alias to use template instance with default allocator
using LandingTarget =
  mavros_msgs::msg::LandingTarget_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::GLOBAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::LOCAL_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::MISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::GLOBAL_RELATIVE_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::LOCAL_ENU;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::GLOBAL_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::GLOBAL_RELATIVE_ALT_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::LOCAL_OFFSET_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::BODY_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::BODY_OFFSET_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::GLOBAL_TERRAIN_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::GLOBAL_TERRAIN_ALT_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::LIGHT_BEACON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::RADIO_BEACON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::VISION_FIDUCIAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LandingTarget_<ContainerAllocator>::VISION_OTHER;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__STRUCT_HPP_
