// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Trajectory_time_horizon
{
public:
  explicit Init_Trajectory_time_horizon(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Trajectory time_horizon(::mavros_msgs::msg::Trajectory::_time_horizon_type arg)
  {
    msg_.time_horizon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_command
{
public:
  explicit Init_Trajectory_command(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_time_horizon command(::mavros_msgs::msg::Trajectory::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_Trajectory_time_horizon(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_point_valid
{
public:
  explicit Init_Trajectory_point_valid(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_command point_valid(::mavros_msgs::msg::Trajectory::_point_valid_type arg)
  {
    msg_.point_valid = std::move(arg);
    return Init_Trajectory_command(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_point_5
{
public:
  explicit Init_Trajectory_point_5(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_point_valid point_5(::mavros_msgs::msg::Trajectory::_point_5_type arg)
  {
    msg_.point_5 = std::move(arg);
    return Init_Trajectory_point_valid(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_point_4
{
public:
  explicit Init_Trajectory_point_4(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_point_5 point_4(::mavros_msgs::msg::Trajectory::_point_4_type arg)
  {
    msg_.point_4 = std::move(arg);
    return Init_Trajectory_point_5(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_point_3
{
public:
  explicit Init_Trajectory_point_3(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_point_4 point_3(::mavros_msgs::msg::Trajectory::_point_3_type arg)
  {
    msg_.point_3 = std::move(arg);
    return Init_Trajectory_point_4(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_point_2
{
public:
  explicit Init_Trajectory_point_2(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_point_3 point_2(::mavros_msgs::msg::Trajectory::_point_2_type arg)
  {
    msg_.point_2 = std::move(arg);
    return Init_Trajectory_point_3(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_point_1
{
public:
  explicit Init_Trajectory_point_1(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_point_2 point_1(::mavros_msgs::msg::Trajectory::_point_1_type arg)
  {
    msg_.point_1 = std::move(arg);
    return Init_Trajectory_point_2(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_type
{
public:
  explicit Init_Trajectory_type(::mavros_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_point_1 type(::mavros_msgs::msg::Trajectory::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Trajectory_point_1(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_header
{
public:
  Init_Trajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_type header(::mavros_msgs::msg::Trajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Trajectory_type(msg_);
  }

private:
  ::mavros_msgs::msg::Trajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Trajectory>()
{
  return mavros_msgs::msg::builder::Init_Trajectory_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_
