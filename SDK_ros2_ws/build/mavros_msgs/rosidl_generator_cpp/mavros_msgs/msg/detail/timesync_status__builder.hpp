// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/TimesyncStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/timesync_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_TimesyncStatus_round_trip_time_ms
{
public:
  explicit Init_TimesyncStatus_round_trip_time_ms(::mavros_msgs::msg::TimesyncStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::TimesyncStatus round_trip_time_ms(::mavros_msgs::msg::TimesyncStatus::_round_trip_time_ms_type arg)
  {
    msg_.round_trip_time_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::TimesyncStatus msg_;
};

class Init_TimesyncStatus_estimated_offset_ns
{
public:
  explicit Init_TimesyncStatus_estimated_offset_ns(::mavros_msgs::msg::TimesyncStatus & msg)
  : msg_(msg)
  {}
  Init_TimesyncStatus_round_trip_time_ms estimated_offset_ns(::mavros_msgs::msg::TimesyncStatus::_estimated_offset_ns_type arg)
  {
    msg_.estimated_offset_ns = std::move(arg);
    return Init_TimesyncStatus_round_trip_time_ms(msg_);
  }

private:
  ::mavros_msgs::msg::TimesyncStatus msg_;
};

class Init_TimesyncStatus_observed_offset_ns
{
public:
  explicit Init_TimesyncStatus_observed_offset_ns(::mavros_msgs::msg::TimesyncStatus & msg)
  : msg_(msg)
  {}
  Init_TimesyncStatus_estimated_offset_ns observed_offset_ns(::mavros_msgs::msg::TimesyncStatus::_observed_offset_ns_type arg)
  {
    msg_.observed_offset_ns = std::move(arg);
    return Init_TimesyncStatus_estimated_offset_ns(msg_);
  }

private:
  ::mavros_msgs::msg::TimesyncStatus msg_;
};

class Init_TimesyncStatus_remote_timestamp_ns
{
public:
  explicit Init_TimesyncStatus_remote_timestamp_ns(::mavros_msgs::msg::TimesyncStatus & msg)
  : msg_(msg)
  {}
  Init_TimesyncStatus_observed_offset_ns remote_timestamp_ns(::mavros_msgs::msg::TimesyncStatus::_remote_timestamp_ns_type arg)
  {
    msg_.remote_timestamp_ns = std::move(arg);
    return Init_TimesyncStatus_observed_offset_ns(msg_);
  }

private:
  ::mavros_msgs::msg::TimesyncStatus msg_;
};

class Init_TimesyncStatus_header
{
public:
  Init_TimesyncStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimesyncStatus_remote_timestamp_ns header(::mavros_msgs::msg::TimesyncStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TimesyncStatus_remote_timestamp_ns(msg_);
  }

private:
  ::mavros_msgs::msg::TimesyncStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::TimesyncStatus>()
{
  return mavros_msgs::msg::builder::Init_TimesyncStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__BUILDER_HPP_
