// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/StatusEvent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/status_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_StatusEvent_sequence
{
public:
  explicit Init_StatusEvent_sequence(::mavros_msgs::msg::StatusEvent & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::StatusEvent sequence(::mavros_msgs::msg::StatusEvent::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::StatusEvent msg_;
};

class Init_StatusEvent_arguments
{
public:
  explicit Init_StatusEvent_arguments(::mavros_msgs::msg::StatusEvent & msg)
  : msg_(msg)
  {}
  Init_StatusEvent_sequence arguments(::mavros_msgs::msg::StatusEvent::_arguments_type arg)
  {
    msg_.arguments = std::move(arg);
    return Init_StatusEvent_sequence(msg_);
  }

private:
  ::mavros_msgs::msg::StatusEvent msg_;
};

class Init_StatusEvent_px4_id
{
public:
  explicit Init_StatusEvent_px4_id(::mavros_msgs::msg::StatusEvent & msg)
  : msg_(msg)
  {}
  Init_StatusEvent_arguments px4_id(::mavros_msgs::msg::StatusEvent::_px4_id_type arg)
  {
    msg_.px4_id = std::move(arg);
    return Init_StatusEvent_arguments(msg_);
  }

private:
  ::mavros_msgs::msg::StatusEvent msg_;
};

class Init_StatusEvent_severity
{
public:
  explicit Init_StatusEvent_severity(::mavros_msgs::msg::StatusEvent & msg)
  : msg_(msg)
  {}
  Init_StatusEvent_px4_id severity(::mavros_msgs::msg::StatusEvent::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return Init_StatusEvent_px4_id(msg_);
  }

private:
  ::mavros_msgs::msg::StatusEvent msg_;
};

class Init_StatusEvent_header
{
public:
  Init_StatusEvent_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StatusEvent_severity header(::mavros_msgs::msg::StatusEvent::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StatusEvent_severity(msg_);
  }

private:
  ::mavros_msgs::msg::StatusEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::StatusEvent>()
{
  return mavros_msgs::msg::builder::Init_StatusEvent_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__STATUS_EVENT__BUILDER_HPP_
