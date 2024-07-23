// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/TerrainReport.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__TerrainReport __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__TerrainReport __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TerrainReport_
{
  using Type = TerrainReport_<ContainerAllocator>;

  explicit TerrainReport_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->spacing = 0;
      this->terrain_height = 0.0f;
      this->current_height = 0.0f;
      this->pending = 0;
      this->loaded = 0;
    }
  }

  explicit TerrainReport_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->spacing = 0;
      this->terrain_height = 0.0f;
      this->current_height = 0.0f;
      this->pending = 0;
      this->loaded = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _spacing_type =
    uint16_t;
  _spacing_type spacing;
  using _terrain_height_type =
    float;
  _terrain_height_type terrain_height;
  using _current_height_type =
    float;
  _current_height_type current_height;
  using _pending_type =
    uint16_t;
  _pending_type pending;
  using _loaded_type =
    uint16_t;
  _loaded_type loaded;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__spacing(
    const uint16_t & _arg)
  {
    this->spacing = _arg;
    return *this;
  }
  Type & set__terrain_height(
    const float & _arg)
  {
    this->terrain_height = _arg;
    return *this;
  }
  Type & set__current_height(
    const float & _arg)
  {
    this->current_height = _arg;
    return *this;
  }
  Type & set__pending(
    const uint16_t & _arg)
  {
    this->pending = _arg;
    return *this;
  }
  Type & set__loaded(
    const uint16_t & _arg)
  {
    this->loaded = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::TerrainReport_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::TerrainReport_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::TerrainReport_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::TerrainReport_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__TerrainReport
    std::shared_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__TerrainReport
    std::shared_ptr<mavros_msgs::msg::TerrainReport_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TerrainReport_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->spacing != other.spacing) {
      return false;
    }
    if (this->terrain_height != other.terrain_height) {
      return false;
    }
    if (this->current_height != other.current_height) {
      return false;
    }
    if (this->pending != other.pending) {
      return false;
    }
    if (this->loaded != other.loaded) {
      return false;
    }
    return true;
  }
  bool operator!=(const TerrainReport_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TerrainReport_

// alias to use template instance with default allocator
using TerrainReport =
  mavros_msgs::msg::TerrainReport_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__STRUCT_HPP_
