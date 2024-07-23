// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/LogData.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LOG_DATA__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__LOG_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/log_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_LogData_data
{
public:
  explicit Init_LogData_data(::mavros_msgs::msg::LogData & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::LogData data(::mavros_msgs::msg::LogData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::LogData msg_;
};

class Init_LogData_offset
{
public:
  explicit Init_LogData_offset(::mavros_msgs::msg::LogData & msg)
  : msg_(msg)
  {}
  Init_LogData_data offset(::mavros_msgs::msg::LogData::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return Init_LogData_data(msg_);
  }

private:
  ::mavros_msgs::msg::LogData msg_;
};

class Init_LogData_id
{
public:
  explicit Init_LogData_id(::mavros_msgs::msg::LogData & msg)
  : msg_(msg)
  {}
  Init_LogData_offset id(::mavros_msgs::msg::LogData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_LogData_offset(msg_);
  }

private:
  ::mavros_msgs::msg::LogData msg_;
};

class Init_LogData_header
{
public:
  Init_LogData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogData_id header(::mavros_msgs::msg::LogData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LogData_id(msg_);
  }

private:
  ::mavros_msgs::msg::LogData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::LogData>()
{
  return mavros_msgs::msg::builder::Init_LogData_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__LOG_DATA__BUILDER_HPP_
