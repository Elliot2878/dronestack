// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/GimbalManagerSetRoi.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerSetRoi_Request_
{
  using Type = GimbalManagerSetRoi_Request_<ContainerAllocator>;

  explicit GimbalManagerSetRoi_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->gimbal_device_id = 0;
      this->latitude = 0.0f;
      this->longitude = 0.0f;
      this->altitude = 0.0f;
      this->pitch_offset = 0.0f;
      this->roll_offset = 0.0f;
      this->yaw_offset = 0.0f;
      this->sysid = 0;
    }
  }

  explicit GimbalManagerSetRoi_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->gimbal_device_id = 0;
      this->latitude = 0.0f;
      this->longitude = 0.0f;
      this->altitude = 0.0f;
      this->pitch_offset = 0.0f;
      this->roll_offset = 0.0f;
      this->yaw_offset = 0.0f;
      this->sysid = 0;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _gimbal_device_id_type =
    uint8_t;
  _gimbal_device_id_type gimbal_device_id;
  using _latitude_type =
    float;
  _latitude_type latitude;
  using _longitude_type =
    float;
  _longitude_type longitude;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _pitch_offset_type =
    float;
  _pitch_offset_type pitch_offset;
  using _roll_offset_type =
    float;
  _roll_offset_type roll_offset;
  using _yaw_offset_type =
    float;
  _yaw_offset_type yaw_offset;
  using _sysid_type =
    uint8_t;
  _sysid_type sysid;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__gimbal_device_id(
    const uint8_t & _arg)
  {
    this->gimbal_device_id = _arg;
    return *this;
  }
  Type & set__latitude(
    const float & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const float & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__pitch_offset(
    const float & _arg)
  {
    this->pitch_offset = _arg;
    return *this;
  }
  Type & set__roll_offset(
    const float & _arg)
  {
    this->roll_offset = _arg;
    return *this;
  }
  Type & set__yaw_offset(
    const float & _arg)
  {
    this->yaw_offset = _arg;
    return *this;
  }
  Type & set__sysid(
    const uint8_t & _arg)
  {
    this->sysid = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ROI_MODE_LOCATION =
    0u;
  static constexpr uint8_t ROI_MODE_WP_NEXT_OFFSET =
    1u;
  static constexpr uint8_t ROI_MODE_SYSID =
    2u;
  static constexpr uint8_t ROI_MODE_NONE =
    3u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerSetRoi_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->gimbal_device_id != other.gimbal_device_id) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->pitch_offset != other.pitch_offset) {
      return false;
    }
    if (this->roll_offset != other.roll_offset) {
      return false;
    }
    if (this->yaw_offset != other.yaw_offset) {
      return false;
    }
    if (this->sysid != other.sysid) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerSetRoi_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerSetRoi_Request_

// alias to use template instance with default allocator
using GimbalManagerSetRoi_Request =
  mavros_msgs::srv::GimbalManagerSetRoi_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerSetRoi_Request_<ContainerAllocator>::ROI_MODE_LOCATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerSetRoi_Request_<ContainerAllocator>::ROI_MODE_WP_NEXT_OFFSET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerSetRoi_Request_<ContainerAllocator>::ROI_MODE_SYSID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerSetRoi_Request_<ContainerAllocator>::ROI_MODE_NONE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerSetRoi_Response_
{
  using Type = GimbalManagerSetRoi_Response_<ContainerAllocator>;

  explicit GimbalManagerSetRoi_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  explicit GimbalManagerSetRoi_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _result_type =
    uint8_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerSetRoi_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerSetRoi_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerSetRoi_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerSetRoi_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerSetRoi_Response_

// alias to use template instance with default allocator
using GimbalManagerSetRoi_Response =
  mavros_msgs::srv::GimbalManagerSetRoi_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct GimbalManagerSetRoi
{
  using Request = mavros_msgs::srv::GimbalManagerSetRoi_Request;
  using Response = mavros_msgs::srv::GimbalManagerSetRoi_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__STRUCT_HPP_
