// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/GPSRAW.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSRAW__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GPSRAW__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__GPSRAW __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__GPSRAW __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSRAW_
{
  using Type = GPSRAW_<ContainerAllocator>;

  explicit GPSRAW_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->lat = 0l;
      this->lon = 0l;
      this->alt = 0l;
      this->eph = 0;
      this->epv = 0;
      this->vel = 0;
      this->cog = 0;
      this->satellites_visible = 0;
      this->alt_ellipsoid = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
      this->vel_acc = 0ul;
      this->hdg_acc = 0l;
      this->yaw = 0;
      this->dgps_numch = 0;
      this->dgps_age = 0ul;
    }
  }

  explicit GPSRAW_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->lat = 0l;
      this->lon = 0l;
      this->alt = 0l;
      this->eph = 0;
      this->epv = 0;
      this->vel = 0;
      this->cog = 0;
      this->satellites_visible = 0;
      this->alt_ellipsoid = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
      this->vel_acc = 0ul;
      this->hdg_acc = 0l;
      this->yaw = 0;
      this->dgps_numch = 0;
      this->dgps_age = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;
  using _lat_type =
    int32_t;
  _lat_type lat;
  using _lon_type =
    int32_t;
  _lon_type lon;
  using _alt_type =
    int32_t;
  _alt_type alt;
  using _eph_type =
    uint16_t;
  _eph_type eph;
  using _epv_type =
    uint16_t;
  _epv_type epv;
  using _vel_type =
    uint16_t;
  _vel_type vel;
  using _cog_type =
    uint16_t;
  _cog_type cog;
  using _satellites_visible_type =
    uint8_t;
  _satellites_visible_type satellites_visible;
  using _alt_ellipsoid_type =
    int32_t;
  _alt_ellipsoid_type alt_ellipsoid;
  using _h_acc_type =
    uint32_t;
  _h_acc_type h_acc;
  using _v_acc_type =
    uint32_t;
  _v_acc_type v_acc;
  using _vel_acc_type =
    uint32_t;
  _vel_acc_type vel_acc;
  using _hdg_acc_type =
    int32_t;
  _hdg_acc_type hdg_acc;
  using _yaw_type =
    uint16_t;
  _yaw_type yaw;
  using _dgps_numch_type =
    uint8_t;
  _dgps_numch_type dgps_numch;
  using _dgps_age_type =
    uint32_t;
  _dgps_age_type dgps_age;

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
    const int32_t & _arg)
  {
    this->alt = _arg;
    return *this;
  }
  Type & set__eph(
    const uint16_t & _arg)
  {
    this->eph = _arg;
    return *this;
  }
  Type & set__epv(
    const uint16_t & _arg)
  {
    this->epv = _arg;
    return *this;
  }
  Type & set__vel(
    const uint16_t & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__cog(
    const uint16_t & _arg)
  {
    this->cog = _arg;
    return *this;
  }
  Type & set__satellites_visible(
    const uint8_t & _arg)
  {
    this->satellites_visible = _arg;
    return *this;
  }
  Type & set__alt_ellipsoid(
    const int32_t & _arg)
  {
    this->alt_ellipsoid = _arg;
    return *this;
  }
  Type & set__h_acc(
    const uint32_t & _arg)
  {
    this->h_acc = _arg;
    return *this;
  }
  Type & set__v_acc(
    const uint32_t & _arg)
  {
    this->v_acc = _arg;
    return *this;
  }
  Type & set__vel_acc(
    const uint32_t & _arg)
  {
    this->vel_acc = _arg;
    return *this;
  }
  Type & set__hdg_acc(
    const int32_t & _arg)
  {
    this->hdg_acc = _arg;
    return *this;
  }
  Type & set__yaw(
    const uint16_t & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__dgps_numch(
    const uint8_t & _arg)
  {
    this->dgps_numch = _arg;
    return *this;
  }
  Type & set__dgps_age(
    const uint32_t & _arg)
  {
    this->dgps_age = _arg;
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
  static constexpr uint8_t GPS_FIX_TYPE_RTK_FLOAT =
    5u;
  static constexpr uint8_t GPS_FIX_TYPE_RTK_FIXED =
    6u;
  static constexpr uint8_t GPS_FIX_TYPE_STATIC =
    7u;
  static constexpr uint8_t GPS_FIX_TYPE_PPP =
    8u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::GPSRAW_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::GPSRAW_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GPSRAW_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::GPSRAW_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__GPSRAW
    std::shared_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__GPSRAW
    std::shared_ptr<mavros_msgs::msg::GPSRAW_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSRAW_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fix_type != other.fix_type) {
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
    if (this->eph != other.eph) {
      return false;
    }
    if (this->epv != other.epv) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->cog != other.cog) {
      return false;
    }
    if (this->satellites_visible != other.satellites_visible) {
      return false;
    }
    if (this->alt_ellipsoid != other.alt_ellipsoid) {
      return false;
    }
    if (this->h_acc != other.h_acc) {
      return false;
    }
    if (this->v_acc != other.v_acc) {
      return false;
    }
    if (this->vel_acc != other.vel_acc) {
      return false;
    }
    if (this->hdg_acc != other.hdg_acc) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->dgps_numch != other.dgps_numch) {
      return false;
    }
    if (this->dgps_age != other.dgps_age) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSRAW_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSRAW_

// alias to use template instance with default allocator
using GPSRAW =
  mavros_msgs::msg::GPSRAW_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_NO_GPS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_NO_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_2D_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_3D_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_DGPS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_RTK_FLOAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_RTK_FIXED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_STATIC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GPSRAW_<ContainerAllocator>::GPS_FIX_TYPE_PPP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSRAW__STRUCT_HPP_
