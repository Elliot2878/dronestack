// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/GimbalManagerCameraTrack.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerCameraTrack_Request_
{
  using Type = GimbalManagerCameraTrack_Request_<ContainerAllocator>;

  explicit GimbalManagerCameraTrack_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->x = 0.0f;
      this->y = 0.0f;
      this->radius = 0.0f;
      this->top_left_x = 0.0f;
      this->top_left_y = 0.0f;
      this->bottom_right_x = 0.0f;
      this->bottom_right_y = 0.0f;
    }
  }

  explicit GimbalManagerCameraTrack_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->x = 0.0f;
      this->y = 0.0f;
      this->radius = 0.0f;
      this->top_left_x = 0.0f;
      this->top_left_y = 0.0f;
      this->bottom_right_x = 0.0f;
      this->bottom_right_y = 0.0f;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _radius_type =
    float;
  _radius_type radius;
  using _top_left_x_type =
    float;
  _top_left_x_type top_left_x;
  using _top_left_y_type =
    float;
  _top_left_y_type top_left_y;
  using _bottom_right_x_type =
    float;
  _bottom_right_x_type bottom_right_x;
  using _bottom_right_y_type =
    float;
  _bottom_right_y_type bottom_right_y;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__radius(
    const float & _arg)
  {
    this->radius = _arg;
    return *this;
  }
  Type & set__top_left_x(
    const float & _arg)
  {
    this->top_left_x = _arg;
    return *this;
  }
  Type & set__top_left_y(
    const float & _arg)
  {
    this->top_left_y = _arg;
    return *this;
  }
  Type & set__bottom_right_x(
    const float & _arg)
  {
    this->bottom_right_x = _arg;
    return *this;
  }
  Type & set__bottom_right_y(
    const float & _arg)
  {
    this->bottom_right_y = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CAMERA_TRACK_MODE_POINT =
    0u;
  static constexpr uint8_t CAMERA_TRACK_MODE_RECTANGLE =
    1u;
  static constexpr uint8_t CAMERA_TRACK_MODE_STOP_TRACKING =
    2u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerCameraTrack_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->radius != other.radius) {
      return false;
    }
    if (this->top_left_x != other.top_left_x) {
      return false;
    }
    if (this->top_left_y != other.top_left_y) {
      return false;
    }
    if (this->bottom_right_x != other.bottom_right_x) {
      return false;
    }
    if (this->bottom_right_y != other.bottom_right_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerCameraTrack_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerCameraTrack_Request_

// alias to use template instance with default allocator
using GimbalManagerCameraTrack_Request =
  mavros_msgs::srv::GimbalManagerCameraTrack_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerCameraTrack_Request_<ContainerAllocator>::CAMERA_TRACK_MODE_POINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerCameraTrack_Request_<ContainerAllocator>::CAMERA_TRACK_MODE_RECTANGLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GimbalManagerCameraTrack_Request_<ContainerAllocator>::CAMERA_TRACK_MODE_STOP_TRACKING;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerCameraTrack_Response_
{
  using Type = GimbalManagerCameraTrack_Response_<ContainerAllocator>;

  explicit GimbalManagerCameraTrack_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  explicit GimbalManagerCameraTrack_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerCameraTrack_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerCameraTrack_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerCameraTrack_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerCameraTrack_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerCameraTrack_Response_

// alias to use template instance with default allocator
using GimbalManagerCameraTrack_Response =
  mavros_msgs::srv::GimbalManagerCameraTrack_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct GimbalManagerCameraTrack
{
  using Request = mavros_msgs::srv::GimbalManagerCameraTrack_Request;
  using Response = mavros_msgs::srv::GimbalManagerCameraTrack_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__STRUCT_HPP_
