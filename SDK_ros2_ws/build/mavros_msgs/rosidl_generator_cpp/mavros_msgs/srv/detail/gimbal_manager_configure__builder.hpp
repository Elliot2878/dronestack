// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/GimbalManagerConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/gimbal_manager_configure__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerConfigure_Request_gimbal_device_id
{
public:
  explicit Init_GimbalManagerConfigure_Request_gimbal_device_id(::mavros_msgs::srv::GimbalManagerConfigure_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerConfigure_Request gimbal_device_id(::mavros_msgs::srv::GimbalManagerConfigure_Request::_gimbal_device_id_type arg)
  {
    msg_.gimbal_device_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Request msg_;
};

class Init_GimbalManagerConfigure_Request_compid_secondary
{
public:
  explicit Init_GimbalManagerConfigure_Request_compid_secondary(::mavros_msgs::srv::GimbalManagerConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerConfigure_Request_gimbal_device_id compid_secondary(::mavros_msgs::srv::GimbalManagerConfigure_Request::_compid_secondary_type arg)
  {
    msg_.compid_secondary = std::move(arg);
    return Init_GimbalManagerConfigure_Request_gimbal_device_id(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Request msg_;
};

class Init_GimbalManagerConfigure_Request_sysid_secondary
{
public:
  explicit Init_GimbalManagerConfigure_Request_sysid_secondary(::mavros_msgs::srv::GimbalManagerConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerConfigure_Request_compid_secondary sysid_secondary(::mavros_msgs::srv::GimbalManagerConfigure_Request::_sysid_secondary_type arg)
  {
    msg_.sysid_secondary = std::move(arg);
    return Init_GimbalManagerConfigure_Request_compid_secondary(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Request msg_;
};

class Init_GimbalManagerConfigure_Request_compid_primary
{
public:
  explicit Init_GimbalManagerConfigure_Request_compid_primary(::mavros_msgs::srv::GimbalManagerConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerConfigure_Request_sysid_secondary compid_primary(::mavros_msgs::srv::GimbalManagerConfigure_Request::_compid_primary_type arg)
  {
    msg_.compid_primary = std::move(arg);
    return Init_GimbalManagerConfigure_Request_sysid_secondary(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Request msg_;
};

class Init_GimbalManagerConfigure_Request_sysid_primary
{
public:
  Init_GimbalManagerConfigure_Request_sysid_primary()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerConfigure_Request_compid_primary sysid_primary(::mavros_msgs::srv::GimbalManagerConfigure_Request::_sysid_primary_type arg)
  {
    msg_.sysid_primary = std::move(arg);
    return Init_GimbalManagerConfigure_Request_compid_primary(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerConfigure_Request>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerConfigure_Request_sysid_primary();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerConfigure_Response_result
{
public:
  explicit Init_GimbalManagerConfigure_Response_result(::mavros_msgs::srv::GimbalManagerConfigure_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerConfigure_Response result(::mavros_msgs::srv::GimbalManagerConfigure_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Response msg_;
};

class Init_GimbalManagerConfigure_Response_success
{
public:
  Init_GimbalManagerConfigure_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerConfigure_Response_result success(::mavros_msgs::srv::GimbalManagerConfigure_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GimbalManagerConfigure_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerConfigure_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerConfigure_Response>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerConfigure_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__BUILDER_HPP_
