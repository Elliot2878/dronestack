// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/GimbalManagerPitchyaw.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerPitchyaw_Request_
{
  using Type = GimbalManagerPitchyaw_Request_<ContainerAllocator>;

  explicit GimbalManagerPitchyaw_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->pitch_rate = 0.0f;
      this->yaw_rate = 0.0f;
      this->flags = 0ul;
      this->gimbal_device_id = 0;
    }
  }

  explicit GimbalManagerPitchyaw_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->pitch_rate = 0.0f;
      this->yaw_rate = 0.0f;
      this->flags = 0ul;
      this->gimbal_device_id = 0;
    }
  }

  // field types and members
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _pitch_rate_type =
    float;
  _pitch_rate_type pitch_rate;
  using _yaw_rate_type =
    float;
  _yaw_rate_type yaw_rate;
  using _flags_type =
    uint32_t;
  _flags_type flags;
  using _gimbal_device_id_type =
    uint8_t;
  _gimbal_device_id_type gimbal_device_id;

  // setters for named parameter idiom
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch_rate(
    const float & _arg)
  {
    this->pitch_rate = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const float & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }
  Type & set__flags(
    const uint32_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__gimbal_device_id(
    const uint8_t & _arg)
  {
    this->gimbal_device_id = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_RETRACT =
    1u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_NEUTRAL =
    2u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_ROLL_LOCK =
    4u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_PITCH_LOCK =
    8u;
  static constexpr uint32_t GIMBAL_MANAGER_FLAGS_YAW_LOCK =
    16u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerPitchyaw_Request_ & other) const
  {
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch_rate != other.pitch_rate) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    if (this->gimbal_device_id != other.gimbal_device_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerPitchyaw_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerPitchyaw_Request_

// alias to use template instance with default allocator
using GimbalManagerPitchyaw_Request =
  mavros_msgs::srv::GimbalManagerPitchyaw_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerPitchyaw_Request_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_RETRACT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerPitchyaw_Request_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_NEUTRAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerPitchyaw_Request_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_ROLL_LOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerPitchyaw_Request_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_PITCH_LOCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t GimbalManagerPitchyaw_Request_<ContainerAllocator>::GIMBAL_MANAGER_FLAGS_YAW_LOCK;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerPitchyaw_Response_
{
  using Type = GimbalManagerPitchyaw_Response_<ContainerAllocator>;

  explicit GimbalManagerPitchyaw_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  explicit GimbalManagerPitchyaw_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerPitchyaw_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerPitchyaw_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerPitchyaw_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerPitchyaw_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerPitchyaw_Response_

// alias to use template instance with default allocator
using GimbalManagerPitchyaw_Response =
  mavros_msgs::srv::GimbalManagerPitchyaw_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct GimbalManagerPitchyaw
{
  using Request = mavros_msgs::srv::GimbalManagerPitchyaw_Request;
  using Response = mavros_msgs::srv::GimbalManagerPitchyaw_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__STRUCT_HPP_
