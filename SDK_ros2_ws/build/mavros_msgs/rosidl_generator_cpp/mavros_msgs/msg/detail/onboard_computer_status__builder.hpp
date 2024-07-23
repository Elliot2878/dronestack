// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OnboardComputerStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ONBOARD_COMPUTER_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ONBOARD_COMPUTER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/onboard_computer_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OnboardComputerStatus_link_rx_max
{
public:
  explicit Init_OnboardComputerStatus_link_rx_max(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OnboardComputerStatus link_rx_max(::mavros_msgs::msg::OnboardComputerStatus::_link_rx_max_type arg)
  {
    msg_.link_rx_max = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_link_tx_max
{
public:
  explicit Init_OnboardComputerStatus_link_tx_max(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_link_rx_max link_tx_max(::mavros_msgs::msg::OnboardComputerStatus::_link_tx_max_type arg)
  {
    msg_.link_tx_max = std::move(arg);
    return Init_OnboardComputerStatus_link_rx_max(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_link_rx_rate
{
public:
  explicit Init_OnboardComputerStatus_link_rx_rate(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_link_tx_max link_rx_rate(::mavros_msgs::msg::OnboardComputerStatus::_link_rx_rate_type arg)
  {
    msg_.link_rx_rate = std::move(arg);
    return Init_OnboardComputerStatus_link_tx_max(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_link_tx_rate
{
public:
  explicit Init_OnboardComputerStatus_link_tx_rate(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_link_rx_rate link_tx_rate(::mavros_msgs::msg::OnboardComputerStatus::_link_tx_rate_type arg)
  {
    msg_.link_tx_rate = std::move(arg);
    return Init_OnboardComputerStatus_link_rx_rate(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_link_type
{
public:
  explicit Init_OnboardComputerStatus_link_type(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_link_tx_rate link_type(::mavros_msgs::msg::OnboardComputerStatus::_link_type_type arg)
  {
    msg_.link_type = std::move(arg);
    return Init_OnboardComputerStatus_link_tx_rate(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_storage_total
{
public:
  explicit Init_OnboardComputerStatus_storage_total(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_link_type storage_total(::mavros_msgs::msg::OnboardComputerStatus::_storage_total_type arg)
  {
    msg_.storage_total = std::move(arg);
    return Init_OnboardComputerStatus_link_type(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_storage_usage
{
public:
  explicit Init_OnboardComputerStatus_storage_usage(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_storage_total storage_usage(::mavros_msgs::msg::OnboardComputerStatus::_storage_usage_type arg)
  {
    msg_.storage_usage = std::move(arg);
    return Init_OnboardComputerStatus_storage_total(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_storage_type
{
public:
  explicit Init_OnboardComputerStatus_storage_type(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_storage_usage storage_type(::mavros_msgs::msg::OnboardComputerStatus::_storage_type_type arg)
  {
    msg_.storage_type = std::move(arg);
    return Init_OnboardComputerStatus_storage_usage(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_ram_total
{
public:
  explicit Init_OnboardComputerStatus_ram_total(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_storage_type ram_total(::mavros_msgs::msg::OnboardComputerStatus::_ram_total_type arg)
  {
    msg_.ram_total = std::move(arg);
    return Init_OnboardComputerStatus_storage_type(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_ram_usage
{
public:
  explicit Init_OnboardComputerStatus_ram_usage(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_ram_total ram_usage(::mavros_msgs::msg::OnboardComputerStatus::_ram_usage_type arg)
  {
    msg_.ram_usage = std::move(arg);
    return Init_OnboardComputerStatus_ram_total(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_fan_speed
{
public:
  explicit Init_OnboardComputerStatus_fan_speed(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_ram_usage fan_speed(::mavros_msgs::msg::OnboardComputerStatus::_fan_speed_type arg)
  {
    msg_.fan_speed = std::move(arg);
    return Init_OnboardComputerStatus_ram_usage(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_temperature_core
{
public:
  explicit Init_OnboardComputerStatus_temperature_core(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_fan_speed temperature_core(::mavros_msgs::msg::OnboardComputerStatus::_temperature_core_type arg)
  {
    msg_.temperature_core = std::move(arg);
    return Init_OnboardComputerStatus_fan_speed(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_temperature_board
{
public:
  explicit Init_OnboardComputerStatus_temperature_board(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_temperature_core temperature_board(::mavros_msgs::msg::OnboardComputerStatus::_temperature_board_type arg)
  {
    msg_.temperature_board = std::move(arg);
    return Init_OnboardComputerStatus_temperature_core(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_gpu_combined
{
public:
  explicit Init_OnboardComputerStatus_gpu_combined(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_temperature_board gpu_combined(::mavros_msgs::msg::OnboardComputerStatus::_gpu_combined_type arg)
  {
    msg_.gpu_combined = std::move(arg);
    return Init_OnboardComputerStatus_temperature_board(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_gpu_cores
{
public:
  explicit Init_OnboardComputerStatus_gpu_cores(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_gpu_combined gpu_cores(::mavros_msgs::msg::OnboardComputerStatus::_gpu_cores_type arg)
  {
    msg_.gpu_cores = std::move(arg);
    return Init_OnboardComputerStatus_gpu_combined(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_cpu_combined
{
public:
  explicit Init_OnboardComputerStatus_cpu_combined(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_gpu_cores cpu_combined(::mavros_msgs::msg::OnboardComputerStatus::_cpu_combined_type arg)
  {
    msg_.cpu_combined = std::move(arg);
    return Init_OnboardComputerStatus_gpu_cores(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_cpu_cores
{
public:
  explicit Init_OnboardComputerStatus_cpu_cores(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_cpu_combined cpu_cores(::mavros_msgs::msg::OnboardComputerStatus::_cpu_cores_type arg)
  {
    msg_.cpu_cores = std::move(arg);
    return Init_OnboardComputerStatus_cpu_combined(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_type
{
public:
  explicit Init_OnboardComputerStatus_type(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_cpu_cores type(::mavros_msgs::msg::OnboardComputerStatus::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_OnboardComputerStatus_cpu_cores(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_uptime
{
public:
  explicit Init_OnboardComputerStatus_uptime(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_type uptime(::mavros_msgs::msg::OnboardComputerStatus::_uptime_type arg)
  {
    msg_.uptime = std::move(arg);
    return Init_OnboardComputerStatus_type(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_component
{
public:
  explicit Init_OnboardComputerStatus_component(::mavros_msgs::msg::OnboardComputerStatus & msg)
  : msg_(msg)
  {}
  Init_OnboardComputerStatus_uptime component(::mavros_msgs::msg::OnboardComputerStatus::_component_type arg)
  {
    msg_.component = std::move(arg);
    return Init_OnboardComputerStatus_uptime(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

class Init_OnboardComputerStatus_header
{
public:
  Init_OnboardComputerStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OnboardComputerStatus_component header(::mavros_msgs::msg::OnboardComputerStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OnboardComputerStatus_component(msg_);
  }

private:
  ::mavros_msgs::msg::OnboardComputerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OnboardComputerStatus>()
{
  return mavros_msgs::msg::builder::Init_OnboardComputerStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ONBOARD_COMPUTER_STATUS__BUILDER_HPP_
