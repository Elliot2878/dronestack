// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/OpticalFlowRad.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/optical_flow_rad__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpticalFlowRad & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: integration_time_us
  {
    out << "integration_time_us: ";
    rosidl_generator_traits::value_to_yaml(msg.integration_time_us, out);
    out << ", ";
  }

  // member: integrated_x
  {
    out << "integrated_x: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_x, out);
    out << ", ";
  }

  // member: integrated_y
  {
    out << "integrated_y: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_y, out);
    out << ", ";
  }

  // member: integrated_xgyro
  {
    out << "integrated_xgyro: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_xgyro, out);
    out << ", ";
  }

  // member: integrated_ygyro
  {
    out << "integrated_ygyro: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_ygyro, out);
    out << ", ";
  }

  // member: integrated_zgyro
  {
    out << "integrated_zgyro: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_zgyro, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: quality
  {
    out << "quality: ";
    rosidl_generator_traits::value_to_yaml(msg.quality, out);
    out << ", ";
  }

  // member: time_delta_distance_us
  {
    out << "time_delta_distance_us: ";
    rosidl_generator_traits::value_to_yaml(msg.time_delta_distance_us, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OpticalFlowRad & msg,
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

  // member: integration_time_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integration_time_us: ";
    rosidl_generator_traits::value_to_yaml(msg.integration_time_us, out);
    out << "\n";
  }

  // member: integrated_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrated_x: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_x, out);
    out << "\n";
  }

  // member: integrated_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrated_y: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_y, out);
    out << "\n";
  }

  // member: integrated_xgyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrated_xgyro: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_xgyro, out);
    out << "\n";
  }

  // member: integrated_ygyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrated_ygyro: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_ygyro, out);
    out << "\n";
  }

  // member: integrated_zgyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrated_zgyro: ";
    rosidl_generator_traits::value_to_yaml(msg.integrated_zgyro, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
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

  // member: time_delta_distance_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_delta_distance_us: ";
    rosidl_generator_traits::value_to_yaml(msg.time_delta_distance_us, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpticalFlowRad & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::OpticalFlowRad & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::OpticalFlowRad & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::OpticalFlowRad>()
{
  return "mavros_msgs::msg::OpticalFlowRad";
}

template<>
inline const char * name<mavros_msgs::msg::OpticalFlowRad>()
{
  return "mavros_msgs/msg/OpticalFlowRad";
}

template<>
struct has_fixed_size<mavros_msgs::msg::OpticalFlowRad>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::OpticalFlowRad>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::OpticalFlowRad>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__TRAITS_HPP_
