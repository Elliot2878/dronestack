// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/SysStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/sys_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_SysStatus_errors_count4
{
public:
  explicit Init_SysStatus_errors_count4(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::SysStatus errors_count4(::mavros_msgs::msg::SysStatus::_errors_count4_type arg)
  {
    msg_.errors_count4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_errors_count3
{
public:
  explicit Init_SysStatus_errors_count3(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_errors_count4 errors_count3(::mavros_msgs::msg::SysStatus::_errors_count3_type arg)
  {
    msg_.errors_count3 = std::move(arg);
    return Init_SysStatus_errors_count4(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_errors_count2
{
public:
  explicit Init_SysStatus_errors_count2(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_errors_count3 errors_count2(::mavros_msgs::msg::SysStatus::_errors_count2_type arg)
  {
    msg_.errors_count2 = std::move(arg);
    return Init_SysStatus_errors_count3(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_errors_count1
{
public:
  explicit Init_SysStatus_errors_count1(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_errors_count2 errors_count1(::mavros_msgs::msg::SysStatus::_errors_count1_type arg)
  {
    msg_.errors_count1 = std::move(arg);
    return Init_SysStatus_errors_count2(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_errors_comm
{
public:
  explicit Init_SysStatus_errors_comm(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_errors_count1 errors_comm(::mavros_msgs::msg::SysStatus::_errors_comm_type arg)
  {
    msg_.errors_comm = std::move(arg);
    return Init_SysStatus_errors_count1(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_drop_rate_comm
{
public:
  explicit Init_SysStatus_drop_rate_comm(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_errors_comm drop_rate_comm(::mavros_msgs::msg::SysStatus::_drop_rate_comm_type arg)
  {
    msg_.drop_rate_comm = std::move(arg);
    return Init_SysStatus_errors_comm(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_battery_remaining
{
public:
  explicit Init_SysStatus_battery_remaining(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_drop_rate_comm battery_remaining(::mavros_msgs::msg::SysStatus::_battery_remaining_type arg)
  {
    msg_.battery_remaining = std::move(arg);
    return Init_SysStatus_drop_rate_comm(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_current_battery
{
public:
  explicit Init_SysStatus_current_battery(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_battery_remaining current_battery(::mavros_msgs::msg::SysStatus::_current_battery_type arg)
  {
    msg_.current_battery = std::move(arg);
    return Init_SysStatus_battery_remaining(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_voltage_battery
{
public:
  explicit Init_SysStatus_voltage_battery(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_current_battery voltage_battery(::mavros_msgs::msg::SysStatus::_voltage_battery_type arg)
  {
    msg_.voltage_battery = std::move(arg);
    return Init_SysStatus_current_battery(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_load
{
public:
  explicit Init_SysStatus_load(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_voltage_battery load(::mavros_msgs::msg::SysStatus::_load_type arg)
  {
    msg_.load = std::move(arg);
    return Init_SysStatus_voltage_battery(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_sensors_health
{
public:
  explicit Init_SysStatus_sensors_health(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_load sensors_health(::mavros_msgs::msg::SysStatus::_sensors_health_type arg)
  {
    msg_.sensors_health = std::move(arg);
    return Init_SysStatus_load(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_sensors_enabled
{
public:
  explicit Init_SysStatus_sensors_enabled(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_sensors_health sensors_enabled(::mavros_msgs::msg::SysStatus::_sensors_enabled_type arg)
  {
    msg_.sensors_enabled = std::move(arg);
    return Init_SysStatus_sensors_health(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_sensors_present
{
public:
  explicit Init_SysStatus_sensors_present(::mavros_msgs::msg::SysStatus & msg)
  : msg_(msg)
  {}
  Init_SysStatus_sensors_enabled sensors_present(::mavros_msgs::msg::SysStatus::_sensors_present_type arg)
  {
    msg_.sensors_present = std::move(arg);
    return Init_SysStatus_sensors_enabled(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

class Init_SysStatus_header
{
public:
  Init_SysStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SysStatus_sensors_present header(::mavros_msgs::msg::SysStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SysStatus_sensors_present(msg_);
  }

private:
  ::mavros_msgs::msg::SysStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::SysStatus>()
{
  return mavros_msgs::msg::builder::Init_SysStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__BUILDER_HPP_
