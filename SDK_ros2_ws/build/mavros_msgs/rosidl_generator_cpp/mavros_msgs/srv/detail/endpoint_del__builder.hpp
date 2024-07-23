// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/EndpointDel.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__ENDPOINT_DEL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__ENDPOINT_DEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/endpoint_del__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_EndpointDel_Request_type
{
public:
  explicit Init_EndpointDel_Request_type(::mavros_msgs::srv::EndpointDel_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::EndpointDel_Request type(::mavros_msgs::srv::EndpointDel_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointDel_Request msg_;
};

class Init_EndpointDel_Request_url
{
public:
  explicit Init_EndpointDel_Request_url(::mavros_msgs::srv::EndpointDel_Request & msg)
  : msg_(msg)
  {}
  Init_EndpointDel_Request_type url(::mavros_msgs::srv::EndpointDel_Request::_url_type arg)
  {
    msg_.url = std::move(arg);
    return Init_EndpointDel_Request_type(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointDel_Request msg_;
};

class Init_EndpointDel_Request_id
{
public:
  Init_EndpointDel_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndpointDel_Request_url id(::mavros_msgs::srv::EndpointDel_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_EndpointDel_Request_url(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointDel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::EndpointDel_Request>()
{
  return mavros_msgs::srv::builder::Init_EndpointDel_Request_id();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_EndpointDel_Response_successful
{
public:
  Init_EndpointDel_Response_successful()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::EndpointDel_Response successful(::mavros_msgs::srv::EndpointDel_Response::_successful_type arg)
  {
    msg_.successful = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::EndpointDel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::EndpointDel_Response>()
{
  return mavros_msgs::srv::builder::Init_EndpointDel_Response_successful();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__ENDPOINT_DEL__BUILDER_HPP_
