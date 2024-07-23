// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpticalFlow.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/optical_flow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpticalFlow_flow_rate
{
public:
  explicit Init_OpticalFlow_flow_rate(::mavros_msgs::msg::OpticalFlow & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpticalFlow flow_rate(::mavros_msgs::msg::OpticalFlow::_flow_rate_type arg)
  {
    msg_.flow_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlow msg_;
};

class Init_OpticalFlow_ground_distance
{
public:
  explicit Init_OpticalFlow_ground_distance(::mavros_msgs::msg::OpticalFlow & msg)
  : msg_(msg)
  {}
  Init_OpticalFlow_flow_rate ground_distance(::mavros_msgs::msg::OpticalFlow::_ground_distance_type arg)
  {
    msg_.ground_distance = std::move(arg);
    return Init_OpticalFlow_flow_rate(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlow msg_;
};

class Init_OpticalFlow_quality
{
public:
  explicit Init_OpticalFlow_quality(::mavros_msgs::msg::OpticalFlow & msg)
  : msg_(msg)
  {}
  Init_OpticalFlow_ground_distance quality(::mavros_msgs::msg::OpticalFlow::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_OpticalFlow_ground_distance(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlow msg_;
};

class Init_OpticalFlow_flow_comp_m
{
public:
  explicit Init_OpticalFlow_flow_comp_m(::mavros_msgs::msg::OpticalFlow & msg)
  : msg_(msg)
  {}
  Init_OpticalFlow_quality flow_comp_m(::mavros_msgs::msg::OpticalFlow::_flow_comp_m_type arg)
  {
    msg_.flow_comp_m = std::move(arg);
    return Init_OpticalFlow_quality(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlow msg_;
};

class Init_OpticalFlow_flow
{
public:
  explicit Init_OpticalFlow_flow(::mavros_msgs::msg::OpticalFlow & msg)
  : msg_(msg)
  {}
  Init_OpticalFlow_flow_comp_m flow(::mavros_msgs::msg::OpticalFlow::_flow_type arg)
  {
    msg_.flow = std::move(arg);
    return Init_OpticalFlow_flow_comp_m(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlow msg_;
};

class Init_OpticalFlow_header
{
public:
  Init_OpticalFlow_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpticalFlow_flow header(::mavros_msgs::msg::OpticalFlow::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpticalFlow_flow(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlow msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpticalFlow>()
{
  return mavros_msgs::msg::builder::Init_OpticalFlow_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__BUILDER_HPP_
