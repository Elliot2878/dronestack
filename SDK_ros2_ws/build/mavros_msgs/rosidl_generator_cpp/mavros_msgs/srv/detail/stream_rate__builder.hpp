// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/StreamRate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/stream_rate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_StreamRate_Request_on_off
{
public:
  explicit Init_StreamRate_Request_on_off(::mavros_msgs::srv::StreamRate_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::StreamRate_Request on_off(::mavros_msgs::srv::StreamRate_Request::_on_off_type arg)
  {
    msg_.on_off = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::StreamRate_Request msg_;
};

class Init_StreamRate_Request_message_rate
{
public:
  explicit Init_StreamRate_Request_message_rate(::mavros_msgs::srv::StreamRate_Request & msg)
  : msg_(msg)
  {}
  Init_StreamRate_Request_on_off message_rate(::mavros_msgs::srv::StreamRate_Request::_message_rate_type arg)
  {
    msg_.message_rate = std::move(arg);
    return Init_StreamRate_Request_on_off(msg_);
  }

private:
  ::mavros_msgs::srv::StreamRate_Request msg_;
};

class Init_StreamRate_Request_stream_id
{
public:
  Init_StreamRate_Request_stream_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StreamRate_Request_message_rate stream_id(::mavros_msgs::srv::StreamRate_Request::_stream_id_type arg)
  {
    msg_.stream_id = std::move(arg);
    return Init_StreamRate_Request_message_rate(msg_);
  }

private:
  ::mavros_msgs::srv::StreamRate_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::StreamRate_Request>()
{
  return mavros_msgs::srv::builder::Init_StreamRate_Request_stream_id();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::StreamRate_Response>()
{
  return ::mavros_msgs::srv::StreamRate_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__BUILDER_HPP_
