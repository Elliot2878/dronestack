// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/estimator_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_EstimatorStatus_accel_error_status_flag
{
public:
  explicit Init_EstimatorStatus_accel_error_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::EstimatorStatus accel_error_status_flag(::mavros_msgs::msg::EstimatorStatus::_accel_error_status_flag_type arg)
  {
    msg_.accel_error_status_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_gps_glitch_status_flag
{
public:
  explicit Init_EstimatorStatus_gps_glitch_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_accel_error_status_flag gps_glitch_status_flag(::mavros_msgs::msg::EstimatorStatus::_gps_glitch_status_flag_type arg)
  {
    msg_.gps_glitch_status_flag = std::move(arg);
    return Init_EstimatorStatus_accel_error_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_pred_pos_horiz_abs_status_flag
{
public:
  explicit Init_EstimatorStatus_pred_pos_horiz_abs_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_gps_glitch_status_flag pred_pos_horiz_abs_status_flag(::mavros_msgs::msg::EstimatorStatus::_pred_pos_horiz_abs_status_flag_type arg)
  {
    msg_.pred_pos_horiz_abs_status_flag = std::move(arg);
    return Init_EstimatorStatus_gps_glitch_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_pred_pos_horiz_rel_status_flag
{
public:
  explicit Init_EstimatorStatus_pred_pos_horiz_rel_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_pred_pos_horiz_abs_status_flag pred_pos_horiz_rel_status_flag(::mavros_msgs::msg::EstimatorStatus::_pred_pos_horiz_rel_status_flag_type arg)
  {
    msg_.pred_pos_horiz_rel_status_flag = std::move(arg);
    return Init_EstimatorStatus_pred_pos_horiz_abs_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_const_pos_mode_status_flag
{
public:
  explicit Init_EstimatorStatus_const_pos_mode_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_pred_pos_horiz_rel_status_flag const_pos_mode_status_flag(::mavros_msgs::msg::EstimatorStatus::_const_pos_mode_status_flag_type arg)
  {
    msg_.const_pos_mode_status_flag = std::move(arg);
    return Init_EstimatorStatus_pred_pos_horiz_rel_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_pos_vert_agl_status_flag
{
public:
  explicit Init_EstimatorStatus_pos_vert_agl_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_const_pos_mode_status_flag pos_vert_agl_status_flag(::mavros_msgs::msg::EstimatorStatus::_pos_vert_agl_status_flag_type arg)
  {
    msg_.pos_vert_agl_status_flag = std::move(arg);
    return Init_EstimatorStatus_const_pos_mode_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_pos_vert_abs_status_flag
{
public:
  explicit Init_EstimatorStatus_pos_vert_abs_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_pos_vert_agl_status_flag pos_vert_abs_status_flag(::mavros_msgs::msg::EstimatorStatus::_pos_vert_abs_status_flag_type arg)
  {
    msg_.pos_vert_abs_status_flag = std::move(arg);
    return Init_EstimatorStatus_pos_vert_agl_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_pos_horiz_abs_status_flag
{
public:
  explicit Init_EstimatorStatus_pos_horiz_abs_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_pos_vert_abs_status_flag pos_horiz_abs_status_flag(::mavros_msgs::msg::EstimatorStatus::_pos_horiz_abs_status_flag_type arg)
  {
    msg_.pos_horiz_abs_status_flag = std::move(arg);
    return Init_EstimatorStatus_pos_vert_abs_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_pos_horiz_rel_status_flag
{
public:
  explicit Init_EstimatorStatus_pos_horiz_rel_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_pos_horiz_abs_status_flag pos_horiz_rel_status_flag(::mavros_msgs::msg::EstimatorStatus::_pos_horiz_rel_status_flag_type arg)
  {
    msg_.pos_horiz_rel_status_flag = std::move(arg);
    return Init_EstimatorStatus_pos_horiz_abs_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_velocity_vert_status_flag
{
public:
  explicit Init_EstimatorStatus_velocity_vert_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_pos_horiz_rel_status_flag velocity_vert_status_flag(::mavros_msgs::msg::EstimatorStatus::_velocity_vert_status_flag_type arg)
  {
    msg_.velocity_vert_status_flag = std::move(arg);
    return Init_EstimatorStatus_pos_horiz_rel_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_velocity_horiz_status_flag
{
public:
  explicit Init_EstimatorStatus_velocity_horiz_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_velocity_vert_status_flag velocity_horiz_status_flag(::mavros_msgs::msg::EstimatorStatus::_velocity_horiz_status_flag_type arg)
  {
    msg_.velocity_horiz_status_flag = std::move(arg);
    return Init_EstimatorStatus_velocity_vert_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_attitude_status_flag
{
public:
  explicit Init_EstimatorStatus_attitude_status_flag(::mavros_msgs::msg::EstimatorStatus & msg)
  : msg_(msg)
  {}
  Init_EstimatorStatus_velocity_horiz_status_flag attitude_status_flag(::mavros_msgs::msg::EstimatorStatus::_attitude_status_flag_type arg)
  {
    msg_.attitude_status_flag = std::move(arg);
    return Init_EstimatorStatus_velocity_horiz_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

class Init_EstimatorStatus_header
{
public:
  Init_EstimatorStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EstimatorStatus_attitude_status_flag header(::mavros_msgs::msg::EstimatorStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EstimatorStatus_attitude_status_flag(msg_);
  }

private:
  ::mavros_msgs::msg::EstimatorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::EstimatorStatus>()
{
  return mavros_msgs::msg::builder::Init_EstimatorStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__BUILDER_HPP_
