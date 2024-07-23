// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/WaypointSetCurrent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WaypointSetCurrent_Request_
{
  using Type = WaypointSetCurrent_Request_<ContainerAllocator>;

  explicit WaypointSetCurrent_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wp_seq = 0;
    }
  }

  explicit WaypointSetCurrent_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wp_seq = 0;
    }
  }

  // field types and members
  using _wp_seq_type =
    uint16_t;
  _wp_seq_type wp_seq;

  // setters for named parameter idiom
  Type & set__wp_seq(
    const uint16_t & _arg)
  {
    this->wp_seq = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Request
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Request
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointSetCurrent_Request_ & other) const
  {
    if (this->wp_seq != other.wp_seq) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointSetCurrent_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointSetCurrent_Request_

// alias to use template instance with default allocator
using WaypointSetCurrent_Request =
  mavros_msgs::srv::WaypointSetCurrent_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WaypointSetCurrent_Response_
{
  using Type = WaypointSetCurrent_Response_<ContainerAllocator>;

  explicit WaypointSetCurrent_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit WaypointSetCurrent_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Response
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__WaypointSetCurrent_Response
    std::shared_ptr<mavros_msgs::srv::WaypointSetCurrent_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointSetCurrent_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointSetCurrent_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointSetCurrent_Response_

// alias to use template instance with default allocator
using WaypointSetCurrent_Response =
  mavros_msgs::srv::WaypointSetCurrent_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct WaypointSetCurrent
{
  using Request = mavros_msgs::srv::WaypointSetCurrent_Request;
  using Response = mavros_msgs::srv::WaypointSetCurrent_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__STRUCT_HPP_
