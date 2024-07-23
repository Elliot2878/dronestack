// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/VehicleInfoGet.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VehicleInfoGet_Request_
{
  using Type = VehicleInfoGet_Request_<ContainerAllocator>;

  explicit VehicleInfoGet_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sysid = 0;
      this->compid = 0;
      this->get_all = false;
    }
  }

  explicit VehicleInfoGet_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sysid = 0;
      this->compid = 0;
      this->get_all = false;
    }
  }

  // field types and members
  using _sysid_type =
    uint8_t;
  _sysid_type sysid;
  using _compid_type =
    uint8_t;
  _compid_type compid;
  using _get_all_type =
    bool;
  _get_all_type get_all;

  // setters for named parameter idiom
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
  Type & set__get_all(
    const bool & _arg)
  {
    this->get_all = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GET_MY_SYSID =
    0u;
  static constexpr uint8_t GET_MY_COMPID =
    0u;

  // pointer types
  using RawPtr =
    mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Request
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Request
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleInfoGet_Request_ & other) const
  {
    if (this->sysid != other.sysid) {
      return false;
    }
    if (this->compid != other.compid) {
      return false;
    }
    if (this->get_all != other.get_all) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleInfoGet_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleInfoGet_Request_

// alias to use template instance with default allocator
using VehicleInfoGet_Request =
  mavros_msgs::srv::VehicleInfoGet_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t VehicleInfoGet_Request_<ContainerAllocator>::GET_MY_SYSID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t VehicleInfoGet_Request_<ContainerAllocator>::GET_MY_COMPID;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mavros_msgs


// Include directives for member types
// Member 'vehicles'
#include "mavros_msgs/msg/detail/vehicle_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VehicleInfoGet_Response_
{
  using Type = VehicleInfoGet_Response_<ContainerAllocator>;

  explicit VehicleInfoGet_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit VehicleInfoGet_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _vehicles_type =
    std::vector<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>>;
  _vehicles_type vehicles;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__vehicles(
    const std::vector<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mavros_msgs::msg::VehicleInfo_<ContainerAllocator>>> & _arg)
  {
    this->vehicles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Response
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__VehicleInfoGet_Response
    std::shared_ptr<mavros_msgs::srv::VehicleInfoGet_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleInfoGet_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->vehicles != other.vehicles) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleInfoGet_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleInfoGet_Response_

// alias to use template instance with default allocator
using VehicleInfoGet_Response =
  mavros_msgs::srv::VehicleInfoGet_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct VehicleInfoGet
{
  using Request = mavros_msgs::srv::VehicleInfoGet_Request;
  using Response = mavros_msgs::srv::VehicleInfoGet_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__STRUCT_HPP_
