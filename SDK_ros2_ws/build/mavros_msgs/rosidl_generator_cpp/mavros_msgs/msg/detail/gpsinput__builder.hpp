// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GPSINPUT.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSINPUT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GPSINPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/gpsinput__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GPSINPUT_yaw
{
public:
  explicit Init_GPSINPUT_yaw(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GPSINPUT yaw(::mavros_msgs::msg::GPSINPUT::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_satellites_visible
{
public:
  explicit Init_GPSINPUT_satellites_visible(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_yaw satellites_visible(::mavros_msgs::msg::GPSINPUT::_satellites_visible_type arg)
  {
    msg_.satellites_visible = std::move(arg);
    return Init_GPSINPUT_yaw(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_vert_accuracy
{
public:
  explicit Init_GPSINPUT_vert_accuracy(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_satellites_visible vert_accuracy(::mavros_msgs::msg::GPSINPUT::_vert_accuracy_type arg)
  {
    msg_.vert_accuracy = std::move(arg);
    return Init_GPSINPUT_satellites_visible(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_horiz_accuracy
{
public:
  explicit Init_GPSINPUT_horiz_accuracy(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_vert_accuracy horiz_accuracy(::mavros_msgs::msg::GPSINPUT::_horiz_accuracy_type arg)
  {
    msg_.horiz_accuracy = std::move(arg);
    return Init_GPSINPUT_vert_accuracy(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_speed_accuracy
{
public:
  explicit Init_GPSINPUT_speed_accuracy(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_horiz_accuracy speed_accuracy(::mavros_msgs::msg::GPSINPUT::_speed_accuracy_type arg)
  {
    msg_.speed_accuracy = std::move(arg);
    return Init_GPSINPUT_horiz_accuracy(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_vd
{
public:
  explicit Init_GPSINPUT_vd(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_speed_accuracy vd(::mavros_msgs::msg::GPSINPUT::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_GPSINPUT_speed_accuracy(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_ve
{
public:
  explicit Init_GPSINPUT_ve(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_vd ve(::mavros_msgs::msg::GPSINPUT::_ve_type arg)
  {
    msg_.ve = std::move(arg);
    return Init_GPSINPUT_vd(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_vn
{
public:
  explicit Init_GPSINPUT_vn(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_ve vn(::mavros_msgs::msg::GPSINPUT::_vn_type arg)
  {
    msg_.vn = std::move(arg);
    return Init_GPSINPUT_ve(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_vdop
{
public:
  explicit Init_GPSINPUT_vdop(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_vn vdop(::mavros_msgs::msg::GPSINPUT::_vdop_type arg)
  {
    msg_.vdop = std::move(arg);
    return Init_GPSINPUT_vn(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_hdop
{
public:
  explicit Init_GPSINPUT_hdop(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_vdop hdop(::mavros_msgs::msg::GPSINPUT::_hdop_type arg)
  {
    msg_.hdop = std::move(arg);
    return Init_GPSINPUT_vdop(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_alt
{
public:
  explicit Init_GPSINPUT_alt(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_hdop alt(::mavros_msgs::msg::GPSINPUT::_alt_type arg)
  {
    msg_.alt = std::move(arg);
    return Init_GPSINPUT_hdop(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_lon
{
public:
  explicit Init_GPSINPUT_lon(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_alt lon(::mavros_msgs::msg::GPSINPUT::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_GPSINPUT_alt(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_lat
{
public:
  explicit Init_GPSINPUT_lat(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_lon lat(::mavros_msgs::msg::GPSINPUT::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_GPSINPUT_lon(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_time_week
{
public:
  explicit Init_GPSINPUT_time_week(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_lat time_week(::mavros_msgs::msg::GPSINPUT::_time_week_type arg)
  {
    msg_.time_week = std::move(arg);
    return Init_GPSINPUT_lat(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_time_week_ms
{
public:
  explicit Init_GPSINPUT_time_week_ms(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_time_week time_week_ms(::mavros_msgs::msg::GPSINPUT::_time_week_ms_type arg)
  {
    msg_.time_week_ms = std::move(arg);
    return Init_GPSINPUT_time_week(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_ignore_flags
{
public:
  explicit Init_GPSINPUT_ignore_flags(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_time_week_ms ignore_flags(::mavros_msgs::msg::GPSINPUT::_ignore_flags_type arg)
  {
    msg_.ignore_flags = std::move(arg);
    return Init_GPSINPUT_time_week_ms(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_gps_id
{
public:
  explicit Init_GPSINPUT_gps_id(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_ignore_flags gps_id(::mavros_msgs::msg::GPSINPUT::_gps_id_type arg)
  {
    msg_.gps_id = std::move(arg);
    return Init_GPSINPUT_ignore_flags(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_fix_type
{
public:
  explicit Init_GPSINPUT_fix_type(::mavros_msgs::msg::GPSINPUT & msg)
  : msg_(msg)
  {}
  Init_GPSINPUT_gps_id fix_type(::mavros_msgs::msg::GPSINPUT::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_GPSINPUT_gps_id(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

class Init_GPSINPUT_header
{
public:
  Init_GPSINPUT_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSINPUT_fix_type header(::mavros_msgs::msg::GPSINPUT::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSINPUT_fix_type(msg_);
  }

private:
  ::mavros_msgs::msg::GPSINPUT msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GPSINPUT>()
{
  return mavros_msgs::msg::builder::Init_GPSINPUT_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSINPUT__BUILDER_HPP_
