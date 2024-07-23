// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__VehicleInfo __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__VehicleInfo __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleInfo_
{
  using Type = VehicleInfo_<ContainerAllocator>;

  explicit VehicleInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->available_info = 0;
      this->sysid = 0;
      this->compid = 0;
      this->autopilot = 0;
      this->type = 0;
      this->system_status = 0;
      this->base_mode = 0;
      this->custom_mode = 0ul;
      this->mode = "";
      this->mode_id = 0ul;
      this->capabilities = 0ull;
      this->flight_sw_version = 0ul;
      this->middleware_sw_version = 0ul;
      this->os_sw_version = 0ul;
      this->board_version = 0ul;
      this->flight_custom_version = "";
      this->vendor_id = 0;
      this->product_id = 0;
      this->uid = 0ull;
    }
  }

  explicit VehicleInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    mode(_alloc),
    flight_custom_version(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->available_info = 0;
      this->sysid = 0;
      this->compid = 0;
      this->autopilot = 0;
      this->type = 0;
      this->system_status = 0;
      this->base_mode = 0;
      this->custom_mode = 0ul;
      this->mode = "";
      this->mode_id = 0ul;
      this->capabilities = 0ull;
      this->flight_sw_version = 0ul;
      this->middleware_sw_version = 0ul;
      this->os_sw_version = 0ul;
      this->board_version = 0ul;
      this->flight_custom_version = "";
      this->vendor_id = 0;
      this->product_id = 0;
      this->uid = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _available_info_type =
    uint8_t;
  _available_info_type available_info;
  using _sysid_type =
    uint8_t;
  _sysid_type sysid;
  using _compid_type =
    uint8_t;
  _compid_type compid;
  using _autopilot_type =
    uint8_t;
  _autopilot_type autopilot;
  using _type_type =
    uint8_t;
  _type_type type;
  using _system_status_type =
    uint8_t;
  _system_status_type system_status;
  using _base_mode_type =
    uint8_t;
  _base_mode_type base_mode;
  using _custom_mode_type =
    uint32_t;
  _custom_mode_type custom_mode;
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _mode_id_type =
    uint32_t;
  _mode_id_type mode_id;
  using _capabilities_type =
    uint64_t;
  _capabilities_type capabilities;
  using _flight_sw_version_type =
    uint32_t;
  _flight_sw_version_type flight_sw_version;
  using _middleware_sw_version_type =
    uint32_t;
  _middleware_sw_version_type middleware_sw_version;
  using _os_sw_version_type =
    uint32_t;
  _os_sw_version_type os_sw_version;
  using _board_version_type =
    uint32_t;
  _board_version_type board_version;
  using _flight_custom_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _flight_custom_version_type flight_custom_version;
  using _vendor_id_type =
    uint16_t;
  _vendor_id_type vendor_id;
  using _product_id_type =
    uint16_t;
  _product_id_type product_id;
  using _uid_type =
    uint64_t;
  _uid_type uid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__available_info(
    const uint8_t & _arg)
  {
    this->available_info = _arg;
    return *this;
  }
  Type & set__sysid(
    const uint8_t & _arg)
  {
    this->sysid = _arg;
    return *this;
  }
  Type & set__compid(
    const uint8_t & _arg)
  {
    this->compid = _arg;
    return *this;
  }
  Type & set__autopilot(
    const uint8_t & _arg)
  {
    this->autopilot = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__system_status(
    const uint8_t & _arg)
  {
    this->system_status = _arg;
    return *this;
  }
  Type & set__base_mode(
    const uint8_t & _arg)
  {
    this->base_mode = _arg;
    return *this;
  }
  Type & set__custom_mode(
    const uint32_t & _arg)
  {
    this->custom_mode = _arg;
    return *this;
  }
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__mode_id(
    const uint32_t & _arg)
  {
    this->mode_id = _arg;
    return *this;
  }
  Type & set__capabilities(
    const uint64_t & _arg)
  {
    this->capabilities = _arg;
    return *this;
  }
  Type & set__flight_sw_version(
    const uint32_t & _arg)
  {
    this->flight_sw_version = _arg;
    return *this;
  }
  Type & set__middleware_sw_version(
    const uint32_t & _arg)
  {
    this->middleware_sw_version = _arg;
    return *this;
  }
  Type & set__os_sw_version(
    const uint32_t & _arg)
  {
    this->os_sw_version = _arg;
    return *this;
  }
  Type & set__board_version(
    const uint32_t & _arg)
  {
    this->board_version = _arg;
    return *this;
  }
  Type & set__flight_custom_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->flight_custom_version = _arg;
    return *this;
  }
  Type & set__vendor_id(
    const uint16_t & _arg)
  {
    this->vendor_id = _arg;
    return *this;
  }
  Type & set__product_id(
    const uint16_t & _arg)
  {
    this->product_id = _arg;
    return *this;
  }
  Type & set__uid(
    const uint64_t & _arg)
  {
    this->uid = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t HAVE_INFO_HEARTBEAT =
    1u;
  static constexpr uint8_t HAVE_INFO_AUTOPILOT_VERSION =
    2u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::VehicleInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::VehicleInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__VehicleInfo
    std::shared_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__VehicleInfo
    std::shared_ptr<mavros_msgs::msg::VehicleInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->available_info != other.available_info) {
      return false;
    }
    if (this->sysid != other.sysid) {
      return false;
    }
    if (this->compid != other.compid) {
      return false;
    }
    if (this->autopilot != other.autopilot) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->system_status != other.system_status) {
      return false;
    }
    if (this->base_mode != other.base_mode) {
      return false;
    }
    if (this->custom_mode != other.custom_mode) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->mode_id != other.mode_id) {
      return false;
    }
    if (this->capabilities != other.capabilities) {
      return false;
    }
    if (this->flight_sw_version != other.flight_sw_version) {
      return false;
    }
    if (this->middleware_sw_version != other.middleware_sw_version) {
      return false;
    }
    if (this->os_sw_version != other.os_sw_version) {
      return false;
    }
    if (this->board_version != other.board_version) {
      return false;
    }
    if (this->flight_custom_version != other.flight_custom_version) {
      return false;
    }
    if (this->vendor_id != other.vendor_id) {
      return false;
    }
    if (this->product_id != other.product_id) {
      return false;
    }
    if (this->uid != other.uid) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleInfo_

// alias to use template instance with default allocator
using VehicleInfo =
  mavros_msgs::msg::VehicleInfo_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t VehicleInfo_<ContainerAllocator>::HAVE_INFO_HEARTBEAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t VehicleInfo_<ContainerAllocator>::HAVE_INFO_AUTOPILOT_VERSION;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__STRUCT_HPP_
