// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GimbalManagerStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/gimbal_manager_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GimbalManagerStatus_compid_secondary
{
public:
  explicit Init_GimbalManagerStatus_compid_secondary(::mavros_msgs::msg::GimbalManagerStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GimbalManagerStatus compid_secondary(::mavros_msgs::msg::GimbalManagerStatus::_compid_secondary_type arg)
  {
    msg_.compid_secondary = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

class Init_GimbalManagerStatus_sysid_secondary
{
public:
  explicit Init_GimbalManagerStatus_sysid_secondary(::mavros_msgs::msg::GimbalManagerStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerStatus_compid_secondary sysid_secondary(::mavros_msgs::msg::GimbalManagerStatus::_sysid_secondary_type arg)
  {
    msg_.sysid_secondary = std::move(arg);
    return Init_GimbalManagerStatus_compid_secondary(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

class Init_GimbalManagerStatus_compid_primary
{
public:
  explicit Init_GimbalManagerStatus_compid_primary(::mavros_msgs::msg::GimbalManagerStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerStatus_sysid_secondary compid_primary(::mavros_msgs::msg::GimbalManagerStatus::_compid_primary_type arg)
  {
    msg_.compid_primary = std::move(arg);
    return Init_GimbalManagerStatus_sysid_secondary(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

class Init_GimbalManagerStatus_sysid_primary
{
public:
  explicit Init_GimbalManagerStatus_sysid_primary(::mavros_msgs::msg::GimbalManagerStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerStatus_compid_primary sysid_primary(::mavros_msgs::msg::GimbalManagerStatus::_sysid_primary_type arg)
  {
    msg_.sysid_primary = std::move(arg);
    return Init_GimbalManagerStatus_compid_primary(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

class Init_GimbalManagerStatus_gimbal_device_id
{
public:
  explicit Init_GimbalManagerStatus_gimbal_device_id(::mavros_msgs::msg::GimbalManagerStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerStatus_sysid_primary gimbal_device_id(::mavros_msgs::msg::GimbalManagerStatus::_gimbal_device_id_type arg)
  {
    msg_.gimbal_device_id = std::move(arg);
    return Init_GimbalManagerStatus_sysid_primary(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

class Init_GimbalManagerStatus_flags
{
public:
  explicit Init_GimbalManagerStatus_flags(::mavros_msgs::msg::GimbalManagerStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerStatus_gimbal_device_id flags(::mavros_msgs::msg::GimbalManagerStatus::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GimbalManagerStatus_gimbal_device_id(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

class Init_GimbalManagerStatus_header
{
public:
  Init_GimbalManagerStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerStatus_flags header(::mavros_msgs::msg::GimbalManagerStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalManagerStatus_flags(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalManagerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GimbalManagerStatus>()
{
  return mavros_msgs::msg::builder::Init_GimbalManagerStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__BUILDER_HPP_
