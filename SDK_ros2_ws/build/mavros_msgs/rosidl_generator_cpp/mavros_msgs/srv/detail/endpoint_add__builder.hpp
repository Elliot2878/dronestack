// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/EndpointAdd.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/endpoint_add__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_EndpointAdd_Request_type
{
public:
  explicit Init_EndpointAdd_Request_type(::mavros_msgs::srv::EndpointAdd_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::EndpointAdd_Request type(::mavros_msgs::srv::EndpointAdd_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointAdd_Request msg_;
};

class Init_EndpointAdd_Request_url
{
public:
  Init_EndpointAdd_Request_url()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndpointAdd_Request_type url(::mavros_msgs::srv::EndpointAdd_Request::_url_type arg)
  {
    msg_.url = std::move(arg);
    return Init_EndpointAdd_Request_type(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointAdd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::EndpointAdd_Request>()
{
  return mavros_msgs::srv::builder::Init_EndpointAdd_Request_url();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_EndpointAdd_Response_id
{
public:
  explicit Init_EndpointAdd_Response_id(::mavros_msgs::srv::EndpointAdd_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::EndpointAdd_Response id(::mavros_msgs::srv::EndpointAdd_Response::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointAdd_Response msg_;
};

class Init_EndpointAdd_Response_reason
{
public:
  explicit Init_EndpointAdd_Response_reason(::mavros_msgs::srv::EndpointAdd_Response & msg)
  : msg_(msg)
  {}
  Init_EndpointAdd_Response_id reason(::mavros_msgs::srv::EndpointAdd_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_EndpointAdd_Response_id(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointAdd_Response msg_;
};

class Init_EndpointAdd_Response_successful
{
public:
  Init_EndpointAdd_Response_successful()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndpointAdd_Response_reason successful(::mavros_msgs::srv::EndpointAdd_Response::_successful_type arg)
  {
    msg_.successful = std::move(arg);
    return Init_EndpointAdd_Response_reason(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointAdd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::EndpointAdd_Response>()
{
  return mavros_msgs::srv::builder::Init_EndpointAdd_Response_successful();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__BUILDER_HPP_
