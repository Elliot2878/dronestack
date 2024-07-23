// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/HilGPS.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_GPS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_GPS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/hil_gps__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_HilGPS_satellites_visible
{
public:
  explicit Init_HilGPS_satellites_visible(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::HilGPS satellites_visible(::mavros_msgs::msg::HilGPS::_satellites_visible_type arg)
  {
    msg_.satellites_visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_cog
{
public:
  explicit Init_HilGPS_cog(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_satellites_visible cog(::mavros_msgs::msg::HilGPS::_cog_type arg)
  {
    msg_.cog = std::move(arg);
    return Init_HilGPS_satellites_visible(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_vd
{
public:
  explicit Init_HilGPS_vd(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_cog vd(::mavros_msgs::msg::HilGPS::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_HilGPS_cog(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_ve
{
public:
  explicit Init_HilGPS_ve(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_vd ve(::mavros_msgs::msg::HilGPS::_ve_type arg)
  {
    msg_.ve = std::move(arg);
    return Init_HilGPS_vd(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_vn
{
public:
  explicit Init_HilGPS_vn(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_ve vn(::mavros_msgs::msg::HilGPS::_vn_type arg)
  {
    msg_.vn = std::move(arg);
    return Init_HilGPS_ve(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_vel
{
public:
  explicit Init_HilGPS_vel(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_vn vel(::mavros_msgs::msg::HilGPS::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_HilGPS_vn(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_epv
{
public:
  explicit Init_HilGPS_epv(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_vel epv(::mavros_msgs::msg::HilGPS::_epv_type arg)
  {
    msg_.epv = std::move(arg);
    return Init_HilGPS_vel(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_eph
{
public:
  explicit Init_HilGPS_eph(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_epv eph(::mavros_msgs::msg::HilGPS::_eph_type arg)
  {
    msg_.eph = std::move(arg);
    return Init_HilGPS_epv(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_geo
{
public:
  explicit Init_HilGPS_geo(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_eph geo(::mavros_msgs::msg::HilGPS::_geo_type arg)
  {
    msg_.geo = std::move(arg);
    return Init_HilGPS_eph(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_fix_type
{
public:
  explicit Init_HilGPS_fix_type(::mavros_msgs::msg::HilGPS & msg)
  : msg_(msg)
  {}
  Init_HilGPS_geo fix_type(::mavros_msgs::msg::HilGPS::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_HilGPS_geo(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

class Init_HilGPS_header
{
public:
  Init_HilGPS_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HilGPS_fix_type header(::mavros_msgs::msg::HilGPS::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HilGPS_fix_type(msg_);
  }

private:
  ::mavros_msgs::msg::HilGPS msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::HilGPS>()
{
  return mavros_msgs::msg::builder::Init_HilGPS_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_GPS__BUILDER_HPP_
