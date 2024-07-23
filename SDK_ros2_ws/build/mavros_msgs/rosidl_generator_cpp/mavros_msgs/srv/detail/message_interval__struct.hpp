// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/MessageInterval.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__MessageInterval_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__MessageInterval_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MessageInterval_Request_
{
  using Type = MessageInterval_Request_<ContainerAllocator>;

  explicit MessageInterval_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_id = 0ul;
      this->message_rate = 0.0f;
    }
  }

  explicit MessageInterval_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_id = 0ul;
      this->message_rate = 0.0f;
    }
  }

  // field types and members
  using _message_id_type =
    uint32_t;
  _message_id_type message_id;
  using _message_rate_type =
    float;
  _message_rate_type message_rate;

  // setters for named parameter idiom
  Type & set__message_id(
    const uint32_t & _arg)
  {
    this->message_id = _arg;
    return *this;
  }
  Type & set__message_rate(
    const float & _arg)
  {
    this->message_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__MessageInterval_Request
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__MessageInterval_Request
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MessageInterval_Request_ & other) const
  {
    if (this->message_id != other.message_id) {
      return false;
    }
    if (this->message_rate != other.message_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const MessageInterval_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MessageInterval_Request_

// alias to use template instance with default allocator
using MessageInterval_Request =
  mavros_msgs::srv::MessageInterval_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__MessageInterval_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__MessageInterval_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MessageInterval_Response_
{
  using Type = MessageInterval_Response_<ContainerAllocator>;

  explicit MessageInterval_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MessageInterval_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__MessageInterval_Response
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__MessageInterval_Response
    std::shared_ptr<mavros_msgs::srv::MessageInterval_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MessageInterval_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MessageInterval_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MessageInterval_Response_

// alias to use template instance with default allocator
using MessageInterval_Response =
  mavros_msgs::srv::MessageInterval_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct MessageInterval
{
  using Request = mavros_msgs::srv::MessageInterval_Request;
  using Response = mavros_msgs::srv::MessageInterval_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__STRUCT_HPP_
