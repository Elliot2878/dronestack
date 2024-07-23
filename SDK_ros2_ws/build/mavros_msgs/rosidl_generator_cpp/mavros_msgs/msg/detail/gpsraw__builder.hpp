// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GPSRAW.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSRAW__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GPSRAW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/gpsraw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GPSRAW_dgps_age
{
public:
  explicit Init_GPSRAW_dgps_age(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GPSRAW dgps_age(::mavros_msgs::msg::GPSRAW::_dgps_age_type arg)
  {
    msg_.dgps_age = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_dgps_numch
{
public:
  explicit Init_GPSRAW_dgps_numch(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_dgps_age dgps_numch(::mavros_msgs::msg::GPSRAW::_dgps_numch_type arg)
  {
    msg_.dgps_numch = std::move(arg);
    return Init_GPSRAW_dgps_age(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_yaw
{
public:
  explicit Init_GPSRAW_yaw(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_dgps_numch yaw(::mavros_msgs::msg::GPSRAW::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GPSRAW_dgps_numch(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_hdg_acc
{
public:
  explicit Init_GPSRAW_hdg_acc(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_yaw hdg_acc(::mavros_msgs::msg::GPSRAW::_hdg_acc_type arg)
  {
    msg_.hdg_acc = std::move(arg);
    return Init_GPSRAW_yaw(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_vel_acc
{
public:
  explicit Init_GPSRAW_vel_acc(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_hdg_acc vel_acc(::mavros_msgs::msg::GPSRAW::_vel_acc_type arg)
  {
    msg_.vel_acc = std::move(arg);
    return Init_GPSRAW_hdg_acc(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_v_acc
{
public:
  explicit Init_GPSRAW_v_acc(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_vel_acc v_acc(::mavros_msgs::msg::GPSRAW::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return Init_GPSRAW_vel_acc(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_h_acc
{
public:
  explicit Init_GPSRAW_h_acc(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_v_acc h_acc(::mavros_msgs::msg::GPSRAW::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_GPSRAW_v_acc(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_alt_ellipsoid
{
public:
  explicit Init_GPSRAW_alt_ellipsoid(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_h_acc alt_ellipsoid(::mavros_msgs::msg::GPSRAW::_alt_ellipsoid_type arg)
  {
    msg_.alt_ellipsoid = std::move(arg);
    return Init_GPSRAW_h_acc(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_satellites_visible
{
public:
  explicit Init_GPSRAW_satellites_visible(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_alt_ellipsoid satellites_visible(::mavros_msgs::msg::GPSRAW::_satellites_visible_type arg)
  {
    msg_.satellites_visible = std::move(arg);
    return Init_GPSRAW_alt_ellipsoid(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_cog
{
public:
  explicit Init_GPSRAW_cog(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_satellites_visible cog(::mavros_msgs::msg::GPSRAW::_cog_type arg)
  {
    msg_.cog = std::move(arg);
    return Init_GPSRAW_satellites_visible(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_vel
{
public:
  explicit Init_GPSRAW_vel(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_cog vel(::mavros_msgs::msg::GPSRAW::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_GPSRAW_cog(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_epv
{
public:
  explicit Init_GPSRAW_epv(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_vel epv(::mavros_msgs::msg::GPSRAW::_epv_type arg)
  {
    msg_.epv = std::move(arg);
    return Init_GPSRAW_vel(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_eph
{
public:
  explicit Init_GPSRAW_eph(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_epv eph(::mavros_msgs::msg::GPSRAW::_eph_type arg)
  {
    msg_.eph = std::move(arg);
    return Init_GPSRAW_epv(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_alt
{
public:
  explicit Init_GPSRAW_alt(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_eph alt(::mavros_msgs::msg::GPSRAW::_alt_type arg)
  {
    msg_.alt = std::move(arg);
    return Init_GPSRAW_eph(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_lon
{
public:
  explicit Init_GPSRAW_lon(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_alt lon(::mavros_msgs::msg::GPSRAW::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_GPSRAW_alt(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_lat
{
public:
  explicit Init_GPSRAW_lat(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_lon lat(::mavros_msgs::msg::GPSRAW::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_GPSRAW_lon(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_fix_type
{
public:
  explicit Init_GPSRAW_fix_type(::mavros_msgs::msg::GPSRAW & msg)
  : msg_(msg)
  {}
  Init_GPSRAW_lat fix_type(::mavros_msgs::msg::GPSRAW::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_GPSRAW_lat(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

class Init_GPSRAW_header
{
public:
  Init_GPSRAW_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSRAW_fix_type header(::mavros_msgs::msg::GPSRAW::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSRAW_fix_type(msg_);
  }

private:
  ::mavros_msgs::msg::GPSRAW msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GPSRAW>()
{
  return mavros_msgs::msg::builder::Init_GPSRAW_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSRAW__BUILDER_HPP_
