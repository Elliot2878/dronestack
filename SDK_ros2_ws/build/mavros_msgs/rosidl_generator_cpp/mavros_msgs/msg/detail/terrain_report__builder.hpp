// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/TerrainReport.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/terrain_report__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_TerrainReport_loaded
{
public:
  explicit Init_TerrainReport_loaded(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::TerrainReport loaded(::mavros_msgs::msg::TerrainReport::_loaded_type arg)
  {
    msg_.loaded = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_pending
{
public:
  explicit Init_TerrainReport_pending(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  Init_TerrainReport_loaded pending(::mavros_msgs::msg::TerrainReport::_pending_type arg)
  {
    msg_.pending = std::move(arg);
    return Init_TerrainReport_loaded(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_current_height
{
public:
  explicit Init_TerrainReport_current_height(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  Init_TerrainReport_pending current_height(::mavros_msgs::msg::TerrainReport::_current_height_type arg)
  {
    msg_.current_height = std::move(arg);
    return Init_TerrainReport_pending(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_terrain_height
{
public:
  explicit Init_TerrainReport_terrain_height(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  Init_TerrainReport_current_height terrain_height(::mavros_msgs::msg::TerrainReport::_terrain_height_type arg)
  {
    msg_.terrain_height = std::move(arg);
    return Init_TerrainReport_current_height(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_spacing
{
public:
  explicit Init_TerrainReport_spacing(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  Init_TerrainReport_terrain_height spacing(::mavros_msgs::msg::TerrainReport::_spacing_type arg)
  {
    msg_.spacing = std::move(arg);
    return Init_TerrainReport_terrain_height(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_longitude
{
public:
  explicit Init_TerrainReport_longitude(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  Init_TerrainReport_spacing longitude(::mavros_msgs::msg::TerrainReport::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_TerrainReport_spacing(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_latitude
{
public:
  explicit Init_TerrainReport_latitude(::mavros_msgs::msg::TerrainReport & msg)
  : msg_(msg)
  {}
  Init_TerrainReport_longitude latitude(::mavros_msgs::msg::TerrainReport::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_TerrainReport_longitude(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

class Init_TerrainReport_header
{
public:
  Init_TerrainReport_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TerrainReport_latitude header(::mavros_msgs::msg::TerrainReport::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TerrainReport_latitude(msg_);
  }

private:
  ::mavros_msgs::msg::TerrainReport msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::TerrainReport>()
{
  return mavros_msgs::msg::builder::Init_TerrainReport_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__BUILDER_HPP_
