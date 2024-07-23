// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/RTKBaseline.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__RTKBaseline __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__RTKBaseline __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RTKBaseline_
{
  using Type = RTKBaseline_<ContainerAllocator>;

  explicit RTKBaseline_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_last_baseline_ms = 0ul;
      this->rtk_receiver_id = 0;
      this->wn = 0;
      this->tow = 0ul;
      this->rtk_health = 0;
      this->rtk_rate = 0;
      this->nsats = 0;
      this->baseline_coords_type = 0;
      this->baseline_a_mm = 0l;
      this->baseline_b_mm = 0l;
      this->baseline_c_mm = 0l;
      this->accuracy = 0ul;
      this->iar_num_hypotheses = 0l;
    }
  }

  explicit RTKBaseline_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_last_baseline_ms = 0ul;
      this->rtk_receiver_id = 0;
      this->wn = 0;
      this->tow = 0ul;
      this->rtk_health = 0;
      this->rtk_rate = 0;
      this->nsats = 0;
      this->baseline_coords_type = 0;
      this->baseline_a_mm = 0l;
      this->baseline_b_mm = 0l;
      this->baseline_c_mm = 0l;
      this->accuracy = 0ul;
      this->iar_num_hypotheses = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_last_baseline_ms_type =
    uint32_t;
  _time_last_baseline_ms_type time_last_baseline_ms;
  using _rtk_receiver_id_type =
    uint8_t;
  _rtk_receiver_id_type rtk_receiver_id;
  using _wn_type =
    uint16_t;
  _wn_type wn;
  using _tow_type =
    uint32_t;
  _tow_type tow;
  using _rtk_health_type =
    uint8_t;
  _rtk_health_type rtk_health;
  using _rtk_rate_type =
    uint8_t;
  _rtk_rate_type rtk_rate;
  using _nsats_type =
    uint8_t;
  _nsats_type nsats;
  using _baseline_coords_type_type =
    uint8_t;
  _baseline_coords_type_type baseline_coords_type;
  using _baseline_a_mm_type =
    int32_t;
  _baseline_a_mm_type baseline_a_mm;
  using _baseline_b_mm_type =
    int32_t;
  _baseline_b_mm_type baseline_b_mm;
  using _baseline_c_mm_type =
    int32_t;
  _baseline_c_mm_type baseline_c_mm;
  using _accuracy_type =
    uint32_t;
  _accuracy_type accuracy;
  using _iar_num_hypotheses_type =
    int32_t;
  _iar_num_hypotheses_type iar_num_hypotheses;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time_last_baseline_ms(
    const uint32_t & _arg)
  {
    this->time_last_baseline_ms = _arg;
    return *this;
  }
  Type & set__rtk_receiver_id(
    const uint8_t & _arg)
  {
    this->rtk_receiver_id = _arg;
    return *this;
  }
  Type & set__wn(
    const uint16_t & _arg)
  {
    this->wn = _arg;
    return *this;
  }
  Type & set__tow(
    const uint32_t & _arg)
  {
    this->tow = _arg;
    return *this;
  }
  Type & set__rtk_health(
    const uint8_t & _arg)
  {
    this->rtk_health = _arg;
    return *this;
  }
  Type & set__rtk_rate(
    const uint8_t & _arg)
  {
    this->rtk_rate = _arg;
    return *this;
  }
  Type & set__nsats(
    const uint8_t & _arg)
  {
    this->nsats = _arg;
    return *this;
  }
  Type & set__baseline_coords_type(
    const uint8_t & _arg)
  {
    this->baseline_coords_type = _arg;
    return *this;
  }
  Type & set__baseline_a_mm(
    const int32_t & _arg)
  {
    this->baseline_a_mm = _arg;
    return *this;
  }
  Type & set__baseline_b_mm(
    const int32_t & _arg)
  {
    this->baseline_b_mm = _arg;
    return *this;
  }
  Type & set__baseline_c_mm(
    const int32_t & _arg)
  {
    this->baseline_c_mm = _arg;
    return *this;
  }
  Type & set__accuracy(
    const uint32_t & _arg)
  {
    this->accuracy = _arg;
    return *this;
  }
  Type & set__iar_num_hypotheses(
    const int32_t & _arg)
  {
    this->iar_num_hypotheses = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t RTK_BASELINE_COORDINATE_SYSTEM_ECEF =
    0u;
  static constexpr uint8_t RTK_BASELINE_COORDINATE_SYSTEM_NED =
    1u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::RTKBaseline_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::RTKBaseline_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::RTKBaseline_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::RTKBaseline_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__RTKBaseline
    std::shared_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__RTKBaseline
    std::shared_ptr<mavros_msgs::msg::RTKBaseline_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RTKBaseline_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_last_baseline_ms != other.time_last_baseline_ms) {
      return false;
    }
    if (this->rtk_receiver_id != other.rtk_receiver_id) {
      return false;
    }
    if (this->wn != other.wn) {
      return false;
    }
    if (this->tow != other.tow) {
      return false;
    }
    if (this->rtk_health != other.rtk_health) {
      return false;
    }
    if (this->rtk_rate != other.rtk_rate) {
      return false;
    }
    if (this->nsats != other.nsats) {
      return false;
    }
    if (this->baseline_coords_type != other.baseline_coords_type) {
      return false;
    }
    if (this->baseline_a_mm != other.baseline_a_mm) {
      return false;
    }
    if (this->baseline_b_mm != other.baseline_b_mm) {
      return false;
    }
    if (this->baseline_c_mm != other.baseline_c_mm) {
      return false;
    }
    if (this->accuracy != other.accuracy) {
      return false;
    }
    if (this->iar_num_hypotheses != other.iar_num_hypotheses) {
      return false;
    }
    return true;
  }
  bool operator!=(const RTKBaseline_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RTKBaseline_

// alias to use template instance with default allocator
using RTKBaseline =
  mavros_msgs::msg::RTKBaseline_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKBaseline_<ContainerAllocator>::RTK_BASELINE_COORDINATE_SYSTEM_ECEF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKBaseline_<ContainerAllocator>::RTK_BASELINE_COORDINATE_SYSTEM_NED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__STRUCT_HPP_
