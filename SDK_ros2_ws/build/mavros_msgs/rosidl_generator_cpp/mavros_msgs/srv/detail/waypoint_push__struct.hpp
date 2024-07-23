// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/WaypointPush.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'waypoints'
#include "mavros_msgs/msg/detail/waypoint__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__WaypointPush_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__WaypointPush_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WaypointPush_Request_
{
  using Type = WaypointPush_Request_<ContainerAllocator>;

  explicit WaypointPush_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_index = 0;
    }
  }

  explicit WaypointPush_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_index = 0;
    }
  }

  // field types and members
  using _start_index_type =
    uint16_t;
  _start_index_type start_index;
  using _waypoints_type =
    std::vector<mavros_msgs::msg::Waypoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mavros_msgs::msg::Waypoint_<ContainerAllocator>>>;
  _waypoints_type waypoints;

  // setters for named parameter idiom
  Type & set__start_index(
    const uint16_t & _arg)
  {
    this->start_index = _arg;
    return *this;
  }
  Type & set__waypoints(
    const std::vector<mavros_msgs::msg::Waypoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mavros_msgs::msg::Waypoint_<ContainerAllocator>>> & _arg)
  {
    this->waypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__WaypointPush_Request
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__WaypointPush_Request
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointPush_Request_ & other) const
  {
    if (this->start_index != other.start_index) {
      return false;
    }
    if (this->waypoints != other.waypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointPush_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointPush_Request_

// alias to use template instance with default allocator
using WaypointPush_Request =
  mavros_msgs::srv::WaypointPush_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__WaypointPush_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__WaypointPush_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WaypointPush_Response_
{
  using Type = WaypointPush_Response_<ContainerAllocator>;

  explicit WaypointPush_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->wp_transfered = 0ul;
    }
  }

  explicit WaypointPush_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->wp_transfered = 0ul;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _wp_transfered_type =
    uint32_t;
  _wp_transfered_type wp_transfered;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__wp_transfered(
    const uint32_t & _arg)
  {
    this->wp_transfered = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__WaypointPush_Response
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__WaypointPush_Response
    std::shared_ptr<mavros_msgs::srv::WaypointPush_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointPush_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->wp_transfered != other.wp_transfered) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointPush_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointPush_Response_

// alias to use template instance with default allocator
using WaypointPush_Response =
  mavros_msgs::srv::WaypointPush_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct WaypointPush
{
  using Request = mavros_msgs::srv::WaypointPush_Request;
  using Response = mavros_msgs::srv::WaypointPush_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__STRUCT_HPP_
