// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:srv/ParamSetV2.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_SET_V2__STRUCT_HPP_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_SET_V2__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'value'
#include "rcl_interfaces/msg/detail/parameter_value__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__ParamSetV2_Request __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__ParamSetV2_Request __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ParamSetV2_Request_
{
  using Type = ParamSetV2_Request_<ContainerAllocator>;

  explicit ParamSetV2_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : value(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force_set = false;
      this->param_id = "";
    }
  }

  explicit ParamSetV2_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : param_id(_alloc),
    value(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force_set = false;
      this->param_id = "";
    }
  }

  // field types and members
  using _force_set_type =
    bool;
  _force_set_type force_set;
  using _param_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _param_id_type param_id;
  using _value_type =
    rcl_interfaces::msg::ParameterValue_<ContainerAllocator>;
  _value_type value;

  // setters for named parameter idiom
  Type & set__force_set(
    const bool & _arg)
  {
    this->force_set = _arg;
    return *this;
  }
  Type & set__param_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->param_id = _arg;
    return *this;
  }
  Type & set__value(
    const rcl_interfaces::msg::ParameterValue_<ContainerAllocator> & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__ParamSetV2_Request
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__ParamSetV2_Request
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParamSetV2_Request_ & other) const
  {
    if (this->force_set != other.force_set) {
      return false;
    }
    if (this->param_id != other.param_id) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParamSetV2_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParamSetV2_Request_

// alias to use template instance with default allocator
using ParamSetV2_Request =
  mavros_msgs::srv::ParamSetV2_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs


// Include directives for member types
// Member 'value'
// already included above
// #include "rcl_interfaces/msg/detail/parameter_value__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__srv__ParamSetV2_Response __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__srv__ParamSetV2_Response __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ParamSetV2_Response_
{
  using Type = ParamSetV2_Response_<ContainerAllocator>;

  explicit ParamSetV2_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : value(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ParamSetV2_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : value(_alloc, _init)
  {
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
  using _value_type =
    rcl_interfaces::msg::ParameterValue_<ContainerAllocator>;
  _value_type value;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__value(
    const rcl_interfaces::msg::ParameterValue_<ContainerAllocator> & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__srv__ParamSetV2_Response
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__srv__ParamSetV2_Response
    std::shared_ptr<mavros_msgs::srv::ParamSetV2_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParamSetV2_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParamSetV2_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParamSetV2_Response_

// alias to use template instance with default allocator
using ParamSetV2_Response =
  mavros_msgs::srv::ParamSetV2_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mavros_msgs

namespace mavros_msgs
{

namespace srv
{

struct ParamSetV2
{
  using Request = mavros_msgs::srv::ParamSetV2_Request;
  using Response = mavros_msgs::srv::ParamSetV2_Response;
};

}  // namespace srv

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_SET_V2__STRUCT_HPP_
