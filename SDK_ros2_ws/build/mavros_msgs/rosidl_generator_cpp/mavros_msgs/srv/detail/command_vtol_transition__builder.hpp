// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandVtolTransition.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_VTOL_TRANSITION__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_VTOL_TRANSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_vtol_transition__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandVtolTransition_Request_state
{
public:
  explicit Init_CommandVtolTransition_Request_state(::mavros_msgs::srv::CommandVtolTransition_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandVtolTransition_Request state(::mavros_msgs::srv::CommandVtolTransition_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandVtolTransition_Request msg_;
};

class Init_CommandVtolTransition_Request_header
{
public:
  Init_CommandVtolTransition_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandVtolTransition_Request_state header(::mavros_msgs::srv::CommandVtolTransition_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CommandVtolTransition_Request_state(msg_);
  }

private:
  ::mavros_msgs::srv::CommandVtolTransition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandVtolTransition_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandVtolTransition_Request_header();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandVtolTransition_Response_result
{
public:
  explicit Init_CommandVtolTransition_Response_result(::mavros_msgs::srv::CommandVtolTransition_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandVtolTransition_Response result(::mavros_msgs::srv::CommandVtolTransition_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandVtolTransition_Response msg_;
};

class Init_CommandVtolTransition_Response_success
{
public:
  Init_CommandVtolTransition_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandVtolTransition_Response_result success(::mavros_msgs::srv::CommandVtolTransition_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandVtolTransition_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandVtolTransition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandVtolTransition_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandVtolTransition_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_VTOL_TRANSITION__BUILDER_HPP_
