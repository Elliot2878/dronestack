// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/SetMavFrame.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__SetMavFrame_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__SetMavFrame_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMavFrame_Request_
{
  using Type = SetMavFrame_Request_<ContainerAllocator>;

  explicit SetMavFrame_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mav_frame = 0;
    }
  }

  explicit SetMavFrame_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mav_frame = 0;
    }
  }

  // field types and members
  using _mav_frame_type =
    uint8_t;
  _mav_frame_type mav_frame;

  // setters for named parameter idiom
  Type & set__mav_frame(
    const uint8_t & _arg)
  {
    this->mav_frame = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FRAME_GLOBAL =
    0u;
  static constexpr uint8_t FRAME_LOCAL_NED =
    1u;
  static constexpr uint8_t FRAME_MISSION =
    2u;
  static constexpr uint8_t FRAME_GLOBAL_RELATIVE_ALT =
    3u;
  static constexpr uint8_t FRAME_LOCAL_ENU =
    4u;
  static constexpr uint8_t FRAME_GLOBAL_INT =
    5u;
  static constexpr uint8_t FRAME_GLOBAL_RELATIVE_ALT_INT =
    6u;
  static constexpr uint8_t FRAME_LOCAL_OFFSET_NED =
    7u;
  static constexpr uint8_t FRAME_BODY_NED =
    8u;
  static constexpr uint8_t FRAME_BODY_OFFSET_NED =
    9u;
  static constexpr uint8_t FRAME_GLOBAL_TERRAIN_ALT =
    10u;
  static constexpr uint8_t FRAME_GLOBAL_TERRAIN_ALT_INT =
    11u;
  static constexpr uint8_t FRAME_BODY_FRD =
    12u;
  static constexpr uint8_t FRAME_RESERVED_13 =
    13u;
  static constexpr uint8_t FRAME_RESERVED_14 =
    14u;
  static constexpr uint8_t FRAME_RESERVED_15 =
    15u;
  static constexpr uint8_t FRAME_RESERVED_16 =
    16u;
  static constexpr uint8_t FRAME_RESERVED_17 =
    17u;
  static constexpr uint8_t FRAME_RESERVED_18 =
    18u;
  static constexpr uint8_t FRAME_RESERVED_19 =
    19u;
  static constexpr uint8_t FRAME_LOCAL_FRD =
    20u;
  static constexpr uint8_t FRAME_LOCAL_FLU =
    21u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__SetMavFrame_Request
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__SetMavFrame_Request
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMavFrame_Request_ & other) const
  {
    if (this->mav_frame != other.mav_frame) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMavFrame_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMavFrame_Request_

// alias to use template instance with default allocator
using SetMavFrame_Request =
  mavros_msgs::srv::SetMavFrame_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_GLOBAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_LOCAL_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_MISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_GLOBAL_RELATIVE_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_LOCAL_ENU;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_GLOBAL_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_GLOBAL_RELATIVE_ALT_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_LOCAL_OFFSET_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_BODY_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_BODY_OFFSET_NED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_GLOBAL_TERRAIN_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_GLOBAL_TERRAIN_ALT_INT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_BODY_FRD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_13;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_14;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_15;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_16;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_17;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_18;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_RESERVED_19;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_LOCAL_FRD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetMavFrame_Request_<ContainerAllocator>::FRAME_LOCAL_FLU;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__SetMavFrame_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__SetMavFrame_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMavFrame_Response_
{
  using Type = SetMavFrame_Response_<ContainerAllocator>;

  explicit SetMavFrame_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetMavFrame_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__SetMavFrame_Response
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__SetMavFrame_Response
    std::shared_ptr<mavros_msgs::srv::SetMavFrame_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMavFrame_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMavFrame_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMavFrame_Response_

// alias to use template instance with default allocator
using SetMavFrame_Response =
  mavros_msgs::srv::SetMavFrame_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct SetMavFrame
{
  using Request = mavros_msgs::srv::SetMavFrame_Request;
  using Response = mavros_msgs::srv::SetMavFrame_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__STRUCT_HPP_
