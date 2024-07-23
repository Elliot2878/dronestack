// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/RTKBaseline.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/rtk_baseline__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_RTKBaseline_iar_num_hypotheses
{
public:
  explicit Init_RTKBaseline_iar_num_hypotheses(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::RTKBaseline iar_num_hypotheses(::mavros_msgs::msg::RTKBaseline::_iar_num_hypotheses_type arg)
  {
    msg_.iar_num_hypotheses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_accuracy
{
public:
  explicit Init_RTKBaseline_accuracy(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_iar_num_hypotheses accuracy(::mavros_msgs::msg::RTKBaseline::_accuracy_type arg)
  {
    msg_.accuracy = std::move(arg);
    return Init_RTKBaseline_iar_num_hypotheses(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_baseline_c_mm
{
public:
  explicit Init_RTKBaseline_baseline_c_mm(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_accuracy baseline_c_mm(::mavros_msgs::msg::RTKBaseline::_baseline_c_mm_type arg)
  {
    msg_.baseline_c_mm = std::move(arg);
    return Init_RTKBaseline_accuracy(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_baseline_b_mm
{
public:
  explicit Init_RTKBaseline_baseline_b_mm(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_baseline_c_mm baseline_b_mm(::mavros_msgs::msg::RTKBaseline::_baseline_b_mm_type arg)
  {
    msg_.baseline_b_mm = std::move(arg);
    return Init_RTKBaseline_baseline_c_mm(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_baseline_a_mm
{
public:
  explicit Init_RTKBaseline_baseline_a_mm(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_baseline_b_mm baseline_a_mm(::mavros_msgs::msg::RTKBaseline::_baseline_a_mm_type arg)
  {
    msg_.baseline_a_mm = std::move(arg);
    return Init_RTKBaseline_baseline_b_mm(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_baseline_coords_type
{
public:
  explicit Init_RTKBaseline_baseline_coords_type(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_baseline_a_mm baseline_coords_type(::mavros_msgs::msg::RTKBaseline::_baseline_coords_type_type arg)
  {
    msg_.baseline_coords_type = std::move(arg);
    return Init_RTKBaseline_baseline_a_mm(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_nsats
{
public:
  explicit Init_RTKBaseline_nsats(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_baseline_coords_type nsats(::mavros_msgs::msg::RTKBaseline::_nsats_type arg)
  {
    msg_.nsats = std::move(arg);
    return Init_RTKBaseline_baseline_coords_type(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_rtk_rate
{
public:
  explicit Init_RTKBaseline_rtk_rate(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_nsats rtk_rate(::mavros_msgs::msg::RTKBaseline::_rtk_rate_type arg)
  {
    msg_.rtk_rate = std::move(arg);
    return Init_RTKBaseline_nsats(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_rtk_health
{
public:
  explicit Init_RTKBaseline_rtk_health(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_rtk_rate rtk_health(::mavros_msgs::msg::RTKBaseline::_rtk_health_type arg)
  {
    msg_.rtk_health = std::move(arg);
    return Init_RTKBaseline_rtk_rate(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_tow
{
public:
  explicit Init_RTKBaseline_tow(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_rtk_health tow(::mavros_msgs::msg::RTKBaseline::_tow_type arg)
  {
    msg_.tow = std::move(arg);
    return Init_RTKBaseline_rtk_health(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_wn
{
public:
  explicit Init_RTKBaseline_wn(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_tow wn(::mavros_msgs::msg::RTKBaseline::_wn_type arg)
  {
    msg_.wn = std::move(arg);
    return Init_RTKBaseline_tow(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_rtk_receiver_id
{
public:
  explicit Init_RTKBaseline_rtk_receiver_id(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_wn rtk_receiver_id(::mavros_msgs::msg::RTKBaseline::_rtk_receiver_id_type arg)
  {
    msg_.rtk_receiver_id = std::move(arg);
    return Init_RTKBaseline_wn(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_time_last_baseline_ms
{
public:
  explicit Init_RTKBaseline_time_last_baseline_ms(::mavros_msgs::msg::RTKBaseline & msg)
  : msg_(msg)
  {}
  Init_RTKBaseline_rtk_receiver_id time_last_baseline_ms(::mavros_msgs::msg::RTKBaseline::_time_last_baseline_ms_type arg)
  {
    msg_.time_last_baseline_ms = std::move(arg);
    return Init_RTKBaseline_rtk_receiver_id(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

class Init_RTKBaseline_header
{
public:
  Init_RTKBaseline_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RTKBaseline_time_last_baseline_ms header(::mavros_msgs::msg::RTKBaseline::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RTKBaseline_time_last_baseline_ms(msg_);
  }

private:
  ::mavros_msgs::msg::RTKBaseline msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::RTKBaseline>()
{
  return mavros_msgs::msg::builder::Init_RTKBaseline_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__BUILDER_HPP_
