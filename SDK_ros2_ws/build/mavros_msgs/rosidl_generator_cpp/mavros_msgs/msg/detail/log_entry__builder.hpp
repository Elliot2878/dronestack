// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/LogEntry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/log_entry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_LogEntry_size
{
public:
  explicit Init_LogEntry_size(::mavros_msgs::msg::LogEntry & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::LogEntry size(::mavros_msgs::msg::LogEntry::_size_type arg)
  {
    msg_.size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::LogEntry msg_;
};

class Init_LogEntry_time_utc
{
public:
  explicit Init_LogEntry_time_utc(::mavros_msgs::msg::LogEntry & msg)
  : msg_(msg)
  {}
  Init_LogEntry_size time_utc(::mavros_msgs::msg::LogEntry::_time_utc_type arg)
  {
    msg_.time_utc = std::move(arg);
    return Init_LogEntry_size(msg_);
  }

private:
  ::mavros_msgs::msg::LogEntry msg_;
};

class Init_LogEntry_last_log_num
{
public:
  explicit Init_LogEntry_last_log_num(::mavros_msgs::msg::LogEntry & msg)
  : msg_(msg)
  {}
  Init_LogEntry_time_utc last_log_num(::mavros_msgs::msg::LogEntry::_last_log_num_type arg)
  {
    msg_.last_log_num = std::move(arg);
    return Init_LogEntry_time_utc(msg_);
  }

private:
  ::mavros_msgs::msg::LogEntry msg_;
};

class Init_LogEntry_num_logs
{
public:
  explicit Init_LogEntry_num_logs(::mavros_msgs::msg::LogEntry & msg)
  : msg_(msg)
  {}
  Init_LogEntry_last_log_num num_logs(::mavros_msgs::msg::LogEntry::_num_logs_type arg)
  {
    msg_.num_logs = std::move(arg);
    return Init_LogEntry_last_log_num(msg_);
  }

private:
  ::mavros_msgs::msg::LogEntry msg_;
};

class Init_LogEntry_id
{
public:
  explicit Init_LogEntry_id(::mavros_msgs::msg::LogEntry & msg)
  : msg_(msg)
  {}
  Init_LogEntry_num_logs id(::mavros_msgs::msg::LogEntry::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_LogEntry_num_logs(msg_);
  }

private:
  ::mavros_msgs::msg::LogEntry msg_;
};

class Init_LogEntry_header
{
public:
  Init_LogEntry_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogEntry_id header(::mavros_msgs::msg::LogEntry::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LogEntry_id(msg_);
  }

private:
  ::mavros_msgs::msg::LogEntry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::LogEntry>()
{
  return mavros_msgs::msg::builder::Init_LogEntry_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__BUILDER_HPP_
