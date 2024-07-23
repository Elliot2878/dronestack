// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/CompanionProcessStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/companion_process_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_CompanionProcessStatus_component
{
public:
  explicit Init_CompanionProcessStatus_component(::mavros_msgs::msg::CompanionProcessStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::CompanionProcessStatus component(::mavros_msgs::msg::CompanionProcessStatus::_component_type arg)
  {
    msg_.component = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::CompanionProcessStatus msg_;
};

class Init_CompanionProcessStatus_state
{
public:
  explicit Init_CompanionProcessStatus_state(::mavros_msgs::msg::CompanionProcessStatus & msg)
  : msg_(msg)
  {}
  Init_CompanionProcessStatus_component state(::mavros_msgs::msg::CompanionProcessStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CompanionProcessStatus_component(msg_);
  }

private:
  ::mavros_msgs::msg::CompanionProcessStatus msg_;
};

class Init_CompanionProcessStatus_header
{
public:
  Init_CompanionProcessStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CompanionProcessStatus_state header(::mavros_msgs::msg::CompanionProcessStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CompanionProcessStatus_state(msg_);
  }

private:
  ::mavros_msgs::msg::CompanionProcessStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::CompanionProcessStatus>()
{
  return mavros_msgs::msg::builder::Init_CompanionProcessStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__COMPANION_PROCESS_STATUS__BUILDER_HPP_
