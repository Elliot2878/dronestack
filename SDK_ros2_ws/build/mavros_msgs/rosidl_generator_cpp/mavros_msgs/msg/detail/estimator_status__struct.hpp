// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__EstimatorStatus __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__EstimatorStatus __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EstimatorStatus_
{
  using Type = EstimatorStatus_<ContainerAllocator>;

  explicit EstimatorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->attitude_status_flag = false;
      this->velocity_horiz_status_flag = false;
      this->velocity_vert_status_flag = false;
      this->pos_horiz_rel_status_flag = false;
      this->pos_horiz_abs_status_flag = false;
      this->pos_vert_abs_status_flag = false;
      this->pos_vert_agl_status_flag = false;
      this->const_pos_mode_status_flag = false;
      this->pred_pos_horiz_rel_status_flag = false;
      this->pred_pos_horiz_abs_status_flag = false;
      this->gps_glitch_status_flag = false;
      this->accel_error_status_flag = false;
    }
  }

  explicit EstimatorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->attitude_status_flag = false;
      this->velocity_horiz_status_flag = false;
      this->velocity_vert_status_flag = false;
      this->pos_horiz_rel_status_flag = false;
      this->pos_horiz_abs_status_flag = false;
      this->pos_vert_abs_status_flag = false;
      this->pos_vert_agl_status_flag = false;
      this->const_pos_mode_status_flag = false;
      this->pred_pos_horiz_rel_status_flag = false;
      this->pred_pos_horiz_abs_status_flag = false;
      this->gps_glitch_status_flag = false;
      this->accel_error_status_flag = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _attitude_status_flag_type =
    bool;
  _attitude_status_flag_type attitude_status_flag;
  using _velocity_horiz_status_flag_type =
    bool;
  _velocity_horiz_status_flag_type velocity_horiz_status_flag;
  using _velocity_vert_status_flag_type =
    bool;
  _velocity_vert_status_flag_type velocity_vert_status_flag;
  using _pos_horiz_rel_status_flag_type =
    bool;
  _pos_horiz_rel_status_flag_type pos_horiz_rel_status_flag;
  using _pos_horiz_abs_status_flag_type =
    bool;
  _pos_horiz_abs_status_flag_type pos_horiz_abs_status_flag;
  using _pos_vert_abs_status_flag_type =
    bool;
  _pos_vert_abs_status_flag_type pos_vert_abs_status_flag;
  using _pos_vert_agl_status_flag_type =
    bool;
  _pos_vert_agl_status_flag_type pos_vert_agl_status_flag;
  using _const_pos_mode_status_flag_type =
    bool;
  _const_pos_mode_status_flag_type const_pos_mode_status_flag;
  using _pred_pos_horiz_rel_status_flag_type =
    bool;
  _pred_pos_horiz_rel_status_flag_type pred_pos_horiz_rel_status_flag;
  using _pred_pos_horiz_abs_status_flag_type =
    bool;
  _pred_pos_horiz_abs_status_flag_type pred_pos_horiz_abs_status_flag;
  using _gps_glitch_status_flag_type =
    bool;
  _gps_glitch_status_flag_type gps_glitch_status_flag;
  using _accel_error_status_flag_type =
    bool;
  _accel_error_status_flag_type accel_error_status_flag;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__attitude_status_flag(
    const bool & _arg)
  {
    this->attitude_status_flag = _arg;
    return *this;
  }
  Type & set__velocity_horiz_status_flag(
    const bool & _arg)
  {
    this->velocity_horiz_status_flag = _arg;
    return *this;
  }
  Type & set__velocity_vert_status_flag(
    const bool & _arg)
  {
    this->velocity_vert_status_flag = _arg;
    return *this;
  }
  Type & set__pos_horiz_rel_status_flag(
    const bool & _arg)
  {
    this->pos_horiz_rel_status_flag = _arg;
    return *this;
  }
  Type & set__pos_horiz_abs_status_flag(
    const bool & _arg)
  {
    this->pos_horiz_abs_status_flag = _arg;
    return *this;
  }
  Type & set__pos_vert_abs_status_flag(
    const bool & _arg)
  {
    this->pos_vert_abs_status_flag = _arg;
    return *this;
  }
  Type & set__pos_vert_agl_status_flag(
    const bool & _arg)
  {
    this->pos_vert_agl_status_flag = _arg;
    return *this;
  }
  Type & set__const_pos_mode_status_flag(
    const bool & _arg)
  {
    this->const_pos_mode_status_flag = _arg;
    return *this;
  }
  Type & set__pred_pos_horiz_rel_status_flag(
    const bool & _arg)
  {
    this->pred_pos_horiz_rel_status_flag = _arg;
    return *this;
  }
  Type & set__pred_pos_horiz_abs_status_flag(
    const bool & _arg)
  {
    this->pred_pos_horiz_abs_status_flag = _arg;
    return *this;
  }
  Type & set__gps_glitch_status_flag(
    const bool & _arg)
  {
    this->gps_glitch_status_flag = _arg;
    return *this;
  }
  Type & set__accel_error_status_flag(
    const bool & _arg)
  {
    this->accel_error_status_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::EstimatorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::EstimatorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::EstimatorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::EstimatorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__EstimatorStatus
    std::shared_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__EstimatorStatus
    std::shared_ptr<mavros_msgs::msg::EstimatorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EstimatorStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->attitude_status_flag != other.attitude_status_flag) {
      return false;
    }
    if (this->velocity_horiz_status_flag != other.velocity_horiz_status_flag) {
      return false;
    }
    if (this->velocity_vert_status_flag != other.velocity_vert_status_flag) {
      return false;
    }
    if (this->pos_horiz_rel_status_flag != other.pos_horiz_rel_status_flag) {
      return false;
    }
    if (this->pos_horiz_abs_status_flag != other.pos_horiz_abs_status_flag) {
      return false;
    }
    if (this->pos_vert_abs_status_flag != other.pos_vert_abs_status_flag) {
      return false;
    }
    if (this->pos_vert_agl_status_flag != other.pos_vert_agl_status_flag) {
      return false;
    }
    if (this->const_pos_mode_status_flag != other.const_pos_mode_status_flag) {
      return false;
    }
    if (this->pred_pos_horiz_rel_status_flag != other.pred_pos_horiz_rel_status_flag) {
      return false;
    }
    if (this->pred_pos_horiz_abs_status_flag != other.pred_pos_horiz_abs_status_flag) {
      return false;
    }
    if (this->gps_glitch_status_flag != other.gps_glitch_status_flag) {
      return false;
    }
    if (this->accel_error_status_flag != other.accel_error_status_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const EstimatorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EstimatorStatus_

// alias to use template instance with default allocator
using EstimatorStatus =
  mavros_msgs::msg::EstimatorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__STRUCT_HPP_
