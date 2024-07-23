// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/OpticalFlow.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/optical_flow__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'flow'
// Member 'flow_comp_m'
// Member 'flow_rate'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpticalFlow & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: flow
  {
    out << "flow: ";
    to_flow_style_yaml(msg.flow, out);
    out << ", ";
  }

  // member: flow_comp_m
  {
    out << "flow_comp_m: ";
    to_flow_style_yaml(msg.flow_comp_m, out);
    out << ", ";
  }

  // member: quality
  {
    out << "quality: ";
    rosidl_generator_traits::value_to_yaml(msg.quality, out);
    out << ", ";
  }

  // member: ground_distance
  {
    out << "ground_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_distance, out);
    out << ", ";
  }

  // member: flow_rate
  {
    out << "flow_rate: ";
    to_flow_style_yaml(msg.flow_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OpticalFlow & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: flow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flow:\n";
    to_block_style_yaml(msg.flow, out, indentation + 2);
  }

  // member: flow_comp_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flow_comp_m:\n";
    to_block_style_yaml(msg.flow_comp_m, out, indentation + 2);
  }

  // member: quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality: ";
    rosidl_generator_traits::value_to_yaml(msg.quality, out);
    out << "\n";
  }

  // member: ground_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ground_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_distance, out);
    out << "\n";
  }

  // member: flow_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flow_rate:\n";
    to_block_style_yaml(msg.flow_rate, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpticalFlow & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::msg::OpticalFlow & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::OpticalFlow & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::OpticalFlow>()
{
  return "mavros_msgs::msg::OpticalFlow";
}

template<>
inline const char * name<mavros_msgs::msg::OpticalFlow>()
{
  return "mavros_msgs/msg/OpticalFlow";
}

template<>
struct has_fixed_size<mavros_msgs::msg::OpticalFlow>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::OpticalFlow>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::OpticalFlow>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW__TRAITS_HPP_
