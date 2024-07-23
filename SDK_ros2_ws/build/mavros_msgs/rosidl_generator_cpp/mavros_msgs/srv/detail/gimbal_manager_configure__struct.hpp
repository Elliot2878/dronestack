// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/GimbalManagerConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerConfigure_Request_
{
  using Type = GimbalManagerConfigure_Request_<ContainerAllocator>;

  explicit GimbalManagerConfigure_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sysid_primary = 0;
      this->compid_primary = 0;
      this->sysid_secondary = 0;
      this->compid_secondary = 0;
      this->gimbal_device_id = 0;
    }
  }

  explicit GimbalManagerConfigure_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sysid_primary = 0;
      this->compid_primary = 0;
      this->sysid_secondary = 0;
      this->compid_secondary = 0;
      this->gimbal_device_id = 0;
    }
  }

  // field types and members
  using _sysid_primary_type =
    int16_t;
  _sysid_primary_type sysid_primary;
  using _compid_primary_type =
    int16_t;
  _compid_primary_type compid_primary;
  using _sysid_secondary_type =
    int16_t;
  _sysid_secondary_type sysid_secondary;
  using _compid_secondary_type =
    int16_t;
  _compid_secondary_type compid_secondary;
  using _gimbal_device_id_type =
    uint8_t;
  _gimbal_device_id_type gimbal_device_id;

  // setters for named parameter idiom
  Type & set__sysid_primary(
    const int16_t & _arg)
  {
    this->sysid_primary = _arg;
    return *this;
  }
  Type & set__compid_primary(
    const int16_t & _arg)
  {
    this->compid_primary = _arg;
    return *this;
  }
  Type & set__sysid_secondary(
    const int16_t & _arg)
  {
    this->sysid_secondary = _arg;
    return *this;
  }
  Type & set__compid_secondary(
    const int16_t & _arg)
  {
    this->compid_secondary = _arg;
    return *this;
  }
  Type & set__gimbal_device_id(
    const uint8_t & _arg)
  {
    this->gimbal_device_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Request
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerConfigure_Request_ & other) const
  {
    if (this->sysid_primary != other.sysid_primary) {
      return false;
    }
    if (this->compid_primary != other.compid_primary) {
      return false;
    }
    if (this->sysid_secondary != other.sysid_secondary) {
      return false;
    }
    if (this->compid_secondary != other.compid_secondary) {
      return false;
    }
    if (this->gimbal_device_id != other.gimbal_device_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerConfigure_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerConfigure_Request_

// alias to use template instance with default allocator
using GimbalManagerConfigure_Request =
  mavros_msgs::srv::GimbalManagerConfigure_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GimbalManagerConfigure_Response_
{
  using Type = GimbalManagerConfigure_Response_<ContainerAllocator>;

  explicit GimbalManagerConfigure_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result = 0;
    }
  }

  explicit GimbalManagerConfigure_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__GimbalManagerConfigure_Response
    std::shared_ptr<mavros_msgs::srv::GimbalManagerConfigure_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalManagerConfigure_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalManagerConfigure_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalManagerConfigure_Response_

// alias to use template instance with default allocator
using GimbalManagerConfigure_Response =
  mavros_msgs::srv::GimbalManagerConfigure_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct GimbalManagerConfigure
{
  using Request = mavros_msgs::srv::GimbalManagerConfigure_Request;
  using Response = mavros_msgs::srv::GimbalManagerConfigure_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__STRUCT_HPP_
