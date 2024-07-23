// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/RadioStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/radio_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_RadioStatus_remrssi_dbm
{
public:
  explicit Init_RadioStatus_remrssi_dbm(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::RadioStatus remrssi_dbm(::mavros_msgs::msg::RadioStatus::_remrssi_dbm_type arg)
  {
    msg_.remrssi_dbm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_rssi_dbm
{
public:
  explicit Init_RadioStatus_rssi_dbm(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_remrssi_dbm rssi_dbm(::mavros_msgs::msg::RadioStatus::_rssi_dbm_type arg)
  {
    msg_.rssi_dbm = std::move(arg);
    return Init_RadioStatus_remrssi_dbm(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_fixed
{
public:
  explicit Init_RadioStatus_fixed(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_rssi_dbm fixed(::mavros_msgs::msg::RadioStatus::_fixed_type arg)
  {
    msg_.fixed = std::move(arg);
    return Init_RadioStatus_rssi_dbm(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_rxerrors
{
public:
  explicit Init_RadioStatus_rxerrors(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_fixed rxerrors(::mavros_msgs::msg::RadioStatus::_rxerrors_type arg)
  {
    msg_.rxerrors = std::move(arg);
    return Init_RadioStatus_fixed(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_remnoise
{
public:
  explicit Init_RadioStatus_remnoise(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_rxerrors remnoise(::mavros_msgs::msg::RadioStatus::_remnoise_type arg)
  {
    msg_.remnoise = std::move(arg);
    return Init_RadioStatus_rxerrors(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_noise
{
public:
  explicit Init_RadioStatus_noise(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_remnoise noise(::mavros_msgs::msg::RadioStatus::_noise_type arg)
  {
    msg_.noise = std::move(arg);
    return Init_RadioStatus_remnoise(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_txbuf
{
public:
  explicit Init_RadioStatus_txbuf(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_noise txbuf(::mavros_msgs::msg::RadioStatus::_txbuf_type arg)
  {
    msg_.txbuf = std::move(arg);
    return Init_RadioStatus_noise(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_remrssi
{
public:
  explicit Init_RadioStatus_remrssi(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_txbuf remrssi(::mavros_msgs::msg::RadioStatus::_remrssi_type arg)
  {
    msg_.remrssi = std::move(arg);
    return Init_RadioStatus_txbuf(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_rssi
{
public:
  explicit Init_RadioStatus_rssi(::mavros_msgs::msg::RadioStatus & msg)
  : msg_(msg)
  {}
  Init_RadioStatus_remrssi rssi(::mavros_msgs::msg::RadioStatus::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_RadioStatus_remrssi(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

class Init_RadioStatus_header
{
public:
  Init_RadioStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadioStatus_rssi header(::mavros_msgs::msg::RadioStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadioStatus_rssi(msg_);
  }

private:
  ::mavros_msgs::msg::RadioStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::RadioStatus>()
{
  return mavros_msgs::msg::builder::Init_RadioStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__BUILDER_HPP_
