// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandInt.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_INT__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_INT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_int__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandInt_Request_z
{
public:
  explicit Init_CommandInt_Request_z(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandInt_Request z(::mavros_msgs::srv::CommandInt_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_y
{
public:
  explicit Init_CommandInt_Request_y(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_z y(::mavros_msgs::srv::CommandInt_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CommandInt_Request_z(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_x
{
public:
  explicit Init_CommandInt_Request_x(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_y x(::mavros_msgs::srv::CommandInt_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CommandInt_Request_y(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_param4
{
public:
  explicit Init_CommandInt_Request_param4(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_x param4(::mavros_msgs::srv::CommandInt_Request::_param4_type arg)
  {
    msg_.param4 = std::move(arg);
    return Init_CommandInt_Request_x(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_param3
{
public:
  explicit Init_CommandInt_Request_param3(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_param4 param3(::mavros_msgs::srv::CommandInt_Request::_param3_type arg)
  {
    msg_.param3 = std::move(arg);
    return Init_CommandInt_Request_param4(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_param2
{
public:
  explicit Init_CommandInt_Request_param2(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_param3 param2(::mavros_msgs::srv::CommandInt_Request::_param2_type arg)
  {
    msg_.param2 = std::move(arg);
    return Init_CommandInt_Request_param3(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_param1
{
public:
  explicit Init_CommandInt_Request_param1(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_param2 param1(::mavros_msgs::srv::CommandInt_Request::_param1_type arg)
  {
    msg_.param1 = std::move(arg);
    return Init_CommandInt_Request_param2(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_autocontinue
{
public:
  explicit Init_CommandInt_Request_autocontinue(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_param1 autocontinue(::mavros_msgs::srv::CommandInt_Request::_autocontinue_type arg)
  {
    msg_.autocontinue = std::move(arg);
    return Init_CommandInt_Request_param1(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_current
{
public:
  explicit Init_CommandInt_Request_current(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_autocontinue current(::mavros_msgs::srv::CommandInt_Request::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_CommandInt_Request_autocontinue(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_command
{
public:
  explicit Init_CommandInt_Request_command(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_current command(::mavros_msgs::srv::CommandInt_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_CommandInt_Request_current(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_frame
{
public:
  explicit Init_CommandInt_Request_frame(::mavros_msgs::srv::CommandInt_Request & msg)
  : msg_(msg)
  {}
  Init_CommandInt_Request_command frame(::mavros_msgs::srv::CommandInt_Request::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_CommandInt_Request_command(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

class Init_CommandInt_Request_broadcast
{
public:
  Init_CommandInt_Request_broadcast()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandInt_Request_frame broadcast(::mavros_msgs::srv::CommandInt_Request::_broadcast_type arg)
  {
    msg_.broadcast = std::move(arg);
    return Init_CommandInt_Request_frame(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandInt_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandInt_Request_broadcast();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandInt_Response_success
{
public:
  Init_CommandInt_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::CommandInt_Response success(::mavros_msgs::srv::CommandInt_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandInt_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandInt_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandInt_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_INT__BUILDER_HPP_
