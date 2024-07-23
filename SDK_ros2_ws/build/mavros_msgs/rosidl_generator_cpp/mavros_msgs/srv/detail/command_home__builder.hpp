// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandHome.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_HOME__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_HOME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_home__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandHome_Request_altitude
{
public:
  explicit Init_CommandHome_Request_altitude(::mavros_msgs::srv::CommandHome_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandHome_Request altitude(::mavros_msgs::srv::CommandHome_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Request msg_;
};

class Init_CommandHome_Request_longitude
{
public:
  explicit Init_CommandHome_Request_longitude(::mavros_msgs::srv::CommandHome_Request & msg)
  : msg_(msg)
  {}
  Init_CommandHome_Request_altitude longitude(::mavros_msgs::srv::CommandHome_Request::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_CommandHome_Request_altitude(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Request msg_;
};

class Init_CommandHome_Request_latitude
{
public:
  explicit Init_CommandHome_Request_latitude(::mavros_msgs::srv::CommandHome_Request & msg)
  : msg_(msg)
  {}
  Init_CommandHome_Request_longitude latitude(::mavros_msgs::srv::CommandHome_Request::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_CommandHome_Request_longitude(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Request msg_;
};

class Init_CommandHome_Request_yaw
{
public:
  explicit Init_CommandHome_Request_yaw(::mavros_msgs::srv::CommandHome_Request & msg)
  : msg_(msg)
  {}
  Init_CommandHome_Request_latitude yaw(::mavros_msgs::srv::CommandHome_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_CommandHome_Request_latitude(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Request msg_;
};

class Init_CommandHome_Request_current_gps
{
public:
  Init_CommandHome_Request_current_gps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandHome_Request_yaw current_gps(::mavros_msgs::srv::CommandHome_Request::_current_gps_type arg)
  {
    msg_.current_gps = std::move(arg);
    return Init_CommandHome_Request_yaw(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandHome_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandHome_Request_current_gps();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandHome_Response_result
{
public:
  explicit Init_CommandHome_Response_result(::mavros_msgs::srv::CommandHome_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandHome_Response result(::mavros_msgs::srv::CommandHome_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Response msg_;
};

class Init_CommandHome_Response_success
{
public:
  Init_CommandHome_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandHome_Response_result success(::mavros_msgs::srv::CommandHome_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandHome_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandHome_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandHome_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandHome_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_HOME__BUILDER_HPP_
