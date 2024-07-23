// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Tunnel.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TUNNEL__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TUNNEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/tunnel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Tunnel_payload
{
public:
  explicit Init_Tunnel_payload(::mavros_msgs::msg::Tunnel & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Tunnel payload(::mavros_msgs::msg::Tunnel::_payload_type arg)
  {
    msg_.payload = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Tunnel msg_;
};

class Init_Tunnel_payload_length
{
public:
  explicit Init_Tunnel_payload_length(::mavros_msgs::msg::Tunnel & msg)
  : msg_(msg)
  {}
  Init_Tunnel_payload payload_length(::mavros_msgs::msg::Tunnel::_payload_length_type arg)
  {
    msg_.payload_length = std::move(arg);
    return Init_Tunnel_payload(msg_);
  }

private:
  ::mavros_msgs::msg::Tunnel msg_;
};

class Init_Tunnel_payload_type
{
public:
  explicit Init_Tunnel_payload_type(::mavros_msgs::msg::Tunnel & msg)
  : msg_(msg)
  {}
  Init_Tunnel_payload_length payload_type(::mavros_msgs::msg::Tunnel::_payload_type_type arg)
  {
    msg_.payload_type = std::move(arg);
    return Init_Tunnel_payload_length(msg_);
  }

private:
  ::mavros_msgs::msg::Tunnel msg_;
};

class Init_Tunnel_target_component
{
public:
  explicit Init_Tunnel_target_component(::mavros_msgs::msg::Tunnel & msg)
  : msg_(msg)
  {}
  Init_Tunnel_payload_type target_component(::mavros_msgs::msg::Tunnel::_target_component_type arg)
  {
    msg_.target_component = std::move(arg);
    return Init_Tunnel_payload_type(msg_);
  }

private:
  ::mavros_msgs::msg::Tunnel msg_;
};

class Init_Tunnel_target_system
{
public:
  Init_Tunnel_target_system()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Tunnel_target_component target_system(::mavros_msgs::msg::Tunnel::_target_system_type arg)
  {
    msg_.target_system = std::move(arg);
    return Init_Tunnel_target_component(msg_);
  }

private:
  ::mavros_msgs::msg::Tunnel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Tunnel>()
{
  return mavros_msgs::msg::builder::Init_Tunnel_target_system();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TUNNEL__BUILDER_HPP_
