// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GPSRTK.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSRTK__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GPSRTK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/gpsrtk__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GPSRTK_iar_num_hypotheses
{
public:
  explicit Init_GPSRTK_iar_num_hypotheses(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GPSRTK iar_num_hypotheses(::mavros_msgs::msg::GPSRTK::_iar_num_hypotheses_type arg)
  {
    msg_.iar_num_hypotheses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_accuracy
{
public:
  explicit Init_GPSRTK_accuracy(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_iar_num_hypotheses accuracy(::mavros_msgs::msg::GPSRTK::_accuracy_type arg)
  {
    msg_.accuracy = std::move(arg);
    return Init_GPSRTK_iar_num_hypotheses(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_baseline_c
{
public:
  explicit Init_GPSRTK_baseline_c(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_accuracy baseline_c(::mavros_msgs::msg::GPSRTK::_baseline_c_type arg)
  {
    msg_.baseline_c = std::move(arg);
    return Init_GPSRTK_accuracy(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_baseline_b
{
public:
  explicit Init_GPSRTK_baseline_b(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_baseline_c baseline_b(::mavros_msgs::msg::GPSRTK::_baseline_b_type arg)
  {
    msg_.baseline_b = std::move(arg);
    return Init_GPSRTK_baseline_c(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_baseline_a
{
public:
  explicit Init_GPSRTK_baseline_a(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_baseline_b baseline_a(::mavros_msgs::msg::GPSRTK::_baseline_a_type arg)
  {
    msg_.baseline_a = std::move(arg);
    return Init_GPSRTK_baseline_b(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_nsats
{
public:
  explicit Init_GPSRTK_nsats(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_baseline_a nsats(::mavros_msgs::msg::GPSRTK::_nsats_type arg)
  {
    msg_.nsats = std::move(arg);
    return Init_GPSRTK_baseline_a(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_rtk_rate
{
public:
  explicit Init_GPSRTK_rtk_rate(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_nsats rtk_rate(::mavros_msgs::msg::GPSRTK::_rtk_rate_type arg)
  {
    msg_.rtk_rate = std::move(arg);
    return Init_GPSRTK_nsats(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_rtk_health
{
public:
  explicit Init_GPSRTK_rtk_health(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_rtk_rate rtk_health(::mavros_msgs::msg::GPSRTK::_rtk_health_type arg)
  {
    msg_.rtk_health = std::move(arg);
    return Init_GPSRTK_rtk_rate(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_tow
{
public:
  explicit Init_GPSRTK_tow(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_rtk_health tow(::mavros_msgs::msg::GPSRTK::_tow_type arg)
  {
    msg_.tow = std::move(arg);
    return Init_GPSRTK_rtk_health(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_wn
{
public:
  explicit Init_GPSRTK_wn(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_tow wn(::mavros_msgs::msg::GPSRTK::_wn_type arg)
  {
    msg_.wn = std::move(arg);
    return Init_GPSRTK_tow(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_rtk_receiver_id
{
public:
  explicit Init_GPSRTK_rtk_receiver_id(::mavros_msgs::msg::GPSRTK & msg)
  : msg_(msg)
  {}
  Init_GPSRTK_wn rtk_receiver_id(::mavros_msgs::msg::GPSRTK::_rtk_receiver_id_type arg)
  {
    msg_.rtk_receiver_id = std::move(arg);
    return Init_GPSRTK_wn(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

class Init_GPSRTK_header
{
public:
  Init_GPSRTK_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSRTK_rtk_receiver_id header(::mavros_msgs::msg::GPSRTK::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSRTK_rtk_receiver_id(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRTK msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GPSRTK>()
{
  return mavros_msgs::msg::builder::Init_GPSRTK_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSRTK__BUILDER_HPP_
