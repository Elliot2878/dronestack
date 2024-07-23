// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/GPSINPUT.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSINPUT__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GPSINPUT__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__GPSINPUT __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__GPSINPUT __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSINPUT_
{
  using Type = GPSINPUT_<ContainerAllocator>;

  explicit GPSINPUT_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->gps_id = 0;
      this->ignore_flags = 0;
      this->time_week_ms = 0ul;
      this->time_week = 0;
      this->lat = 0l;
      this->lon = 0l;
      this->alt = 0.0f;
      this->hdop = 0.0f;
      this->vdop = 0.0f;
      this->vn = 0.0f;
      this->ve = 0.0f;
      this->vd = 0.0f;
      this->speed_accuracy = 0.0f;
      this->horiz_accuracy = 0.0f;
      this->vert_accuracy = 0.0f;
      this->satellites_visible = 0;
      this->yaw = 0;
    }
  }

  explicit GPSINPUT_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->gps_id = 0;
      this->ignore_flags = 0;
      this->time_week_ms = 0ul;
      this->time_week = 0;
      this->lat = 0l;
      this->lon = 0l;
      this->alt = 0.0f;
      this->hdop = 0.0f;
      this->vdop = 0.0f;
      this->vn = 0.0f;
      this->ve = 0.0f;
      this->vd = 0.0f;
      this->speed_accuracy = 0.0f;
      this->horiz_accuracy = 0.0f;
      this->vert_accuracy = 0.0f;
      this->satellites_visible = 0;
      this->yaw = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;
  using _gps_id_type =
    uint8_t;
  _gps_id_type gps_id;
  using _ignore_flags_type =
    uint16_t;
  _ignore_flags_type ignore_flags;
  using _time_week_ms_type =
    uint32_t;
  _time_week_ms_type time_week_ms;
  using _time_week_type =
    uint16_t;
  _time_week_type time_week;
  using _lat_type =
    int32_t;
  _lat_type lat;
  using _lon_type =
    int32_t;
  _lon_type lon;
  using _alt_type =
    float;
  _alt_type alt;
  using _hdop_type =
    float;
  _hdop_type hdop;
  using _vdop_type =
    float;
  _vdop_type vdop;
  using _vn_type =
    float;
  _vn_type vn;
  using _ve_type =
    float;
  _ve_type ve;
  using _vd_type =
    float;
  _vd_type vd;
  using _speed_accuracy_type =
    float;
  _speed_accuracy_type speed_accuracy;
  using _horiz_accuracy_type =
    float;
  _horiz_accuracy_type horiz_accuracy;
  using _vert_accuracy_type =
    float;
  _vert_accuracy_type vert_accuracy;
  using _satellites_visible_type =
    uint8_t;
  _satellites_visible_type satellites_visible;
  using _yaw_type =
    uint16_t;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fix_type(
    const uint8_t & _arg)
  {
    this->fix_type = _arg;
    return *this;
  }
  Type & set__gps_id(
    const uint8_t & _arg)
  {
    this->gps_id = _arg;
    return *this;
  }
  Type & set__ignore_flags(
    const uint16_t & _arg)
  {
    this->ignore_flags = _arg;
    return *this;
  }
  Type & set__time_week_ms(
    const uint32_t & _arg)
  {
    this->time_week_ms = _arg;
    return *this;
  }
  Type & set__time_week(
    const uint16_t & _arg)
  {
    this->time_week = _arg;
    return *this;
  }
  Type & set__lat(
    const int32_t & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__lon(
    const int32_t & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__alt(
    const float & _arg)
  {
    this->alt = _arg;
    return *this;
  }
  Type & set__hdop(
    const float & _arg)
  {
    this->hdop = _arg;
    return *this;
  }
  Type & set__vdop(
    const float & _arg)
  {
    this->vdop = _arg;
    return *this;
  }
  Type & set__vn(
    const float & _arg)
  {
    this->vn = _arg;
    return *this;
  }
  Type & set__ve(
    const float & _arg)
  {
    this->ve = _arg;
    return *this;
  }
  Type & set__vd(
    const float & _arg)
  {
    this->vd = _arg;
    return *this;
  }
  Type & set__speed_accuracy(
    const float & _arg)
  {
    this->speed_accuracy = _arg;
    return *this;
  }
  Type & set__horiz_accuracy(
    const float & _arg)
  {
    this->horiz_accuracy = _arg;
    return *this;
  }
  Type & set__vert_accuracy(
    const float & _arg)
  {
    this->vert_accuracy = _arg;
    return *this;
  }
  Type & set__satellites_visible(
    const uint8_t & _arg)
  {
    this->satellites_visible = _arg;
    return *this;
  }
  Type & set__yaw(
    const uint16_t & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GPS_FIX_TYPE_NO_GPS =
    0u;
  static constexpr uint8_t GPS_FIX_TYPE_NO_FIX =
    1u;
  static constexpr uint8_t GPS_FIX_TYPE_2D_FIX =
    2u;
  static constexpr uint8_t GPS_FIX_TYPE_3D_FIX =
    3u;
  static constexpr uint8_t GPS_FIX_TYPE_DGPS =
    4u;
  static constexpr uint8_t GPS_FIX_TYPE_RTK_FLOATR =
    5u;
  static constexpr uint8_t GPS_FIX_TYPE_RTK_FIXEDR =
    6u;
  static constexpr uint8_t GPS_FIX_TYPE_STATIC =
    7u;
  static constexpr uint8_t GPS_FIX_TYPE_PPP =
    8u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::GPSINPUT_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::GPSINPUT_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GPSINPUT_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GPSINPUT_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__GPSINPUT
    std::shared_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__GPSINPUT
    std::shared_ptr<mavros_msgs::msg::GPSINPUT_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSINPUT_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fix_type != other.fix_type) {
      return false;
    }
    if (this->gps_id != other.gps_id) {
      return false;
    }
    if (this->ignore_flags != other.ignore_flags) {
      return false;
    }
    if (this->time_week_ms != other.time_week_ms) {
      return false;
    }
    if (this->time_week != other.time_week) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->alt != other.alt) {
      return false;
    }
    if (this->hdop != other.hdop) {
      return false;
    }
    if (this->vdop != other.vdop) {
      return false;
    }
    if (this->vn != other.vn) {
      return false;
    }
    if (this->ve != other.ve) {
      return false;
    }
    if (this->vd != other.vd) {
      return false;
    }
    if (this->speed_accuracy != other.speed_accuracy) {
      return false;
    }
    if (this->horiz_accuracy != other.horiz_accuracy) {
      return false;
    }
    if (this->vert_accuracy != other.vert_accuracy) {
      return false;
    }
    if (this->satellites_visible != other.satellites_visible) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSINPUT_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSINPUT_

// alias to use template instance with default allocator
using GPSINPUT =
  mavros_msgs::msg::GPSINPUT_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_NO_GPS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_NO_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_2D_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_3D_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_DGPS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_RTK_FLOATR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_RTK_FIXEDR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_STATIC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSINPUT_<ContainerAllocator>::GPS_FIX_TYPE_PPP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSINPUT__STRUCT_HPP_
