// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/HilGPS.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_GPS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_GPS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__HilGPS __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__HilGPS __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HilGPS_
{
  using Type = HilGPS_<ContainerAllocator>;

  explicit HilGPS_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    geo(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->eph = 0;
      this->epv = 0;
      this->vel = 0;
      this->vn = 0;
      this->ve = 0;
      this->vd = 0;
      this->cog = 0;
      this->satellites_visible = 0;
    }
  }

  explicit HilGPS_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    geo(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->eph = 0;
      this->epv = 0;
      this->vel = 0;
      this->vn = 0;
      this->ve = 0;
      this->vd = 0;
      this->cog = 0;
      this->satellites_visible = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;
  using _geo_type =
    geographic_msgs::msg::GeoPoint_<ContainerAllocator>;
  _geo_type geo;
  using _eph_type =
    uint16_t;
  _eph_type eph;
  using _epv_type =
    uint16_t;
  _epv_type epv;
  using _vel_type =
    uint16_t;
  _vel_type vel;
  using _vn_type =
    int16_t;
  _vn_type vn;
  using _ve_type =
    int16_t;
  _ve_type ve;
  using _vd_type =
    int16_t;
  _vd_type vd;
  using _cog_type =
    uint16_t;
  _cog_type cog;
  using _satellites_visible_type =
    uint8_t;
  _satellites_visible_type satellites_visible;

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
  Type & set__geo(
    const geographic_msgs::msg::GeoPoint_<ContainerAllocator> & _arg)
  {
    this->geo = _arg;
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
  Type & set__vn(
    const int16_t & _arg)
  {
    this->vn = _arg;
    return *this;
  }
  Type & set__ve(
    const int16_t & _arg)
  {
    this->ve = _arg;
    return *this;
  }
  Type & set__vd(
    const int16_t & _arg)
  {
    this->vd = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::HilGPS_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::HilGPS_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilGPS_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilGPS_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__HilGPS
    std::shared_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__HilGPS
    std::shared_ptr<mavros_msgs::msg::HilGPS_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HilGPS_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fix_type != other.fix_type) {
      return false;
    }
    if (this->geo != other.geo) {
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
    if (this->vn != other.vn) {
      return false;
    }
    if (this->ve != other.ve) {
      return false;
    }
    if (this->vd != other.vd) {
      return false;
    }
    if (this->cog != other.cog) {
      return false;
    }
    if (this->satellites_visible != other.satellites_visible) {
      return false;
    }
    return true;
  }
  bool operator!=(const HilGPS_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HilGPS_

// alias to use template instance with default allocator
using HilGPS =
  mavros_msgs::msg::HilGPS_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_GPS__STRUCT_HPP_
