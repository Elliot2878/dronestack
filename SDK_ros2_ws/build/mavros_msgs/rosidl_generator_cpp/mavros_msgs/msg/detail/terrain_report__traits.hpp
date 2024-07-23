// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/TerrainReport.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/terrain_report__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TerrainReport & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: spacing
  {
    out << "spacing: ";
    rosidl_generator_traits::value_to_yaml(msg.spacing, out);
    out << ", ";
  }

  // member: terrain_height
  {
    out << "terrain_height: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_height, out);
    out << ", ";
  }

  // member: current_height
  {
    out << "current_height: ";
    rosidl_generator_traits::value_to_yaml(msg.current_height, out);
    out << ", ";
  }

  // member: pending
  {
    out << "pending: ";
    rosidl_generator_traits::value_to_yaml(msg.pending, out);
    out << ", ";
  }

  // member: loaded
  {
    out << "loaded: ";
    rosidl_generator_traits::value_to_yaml(msg.loaded, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TerrainReport & msg,
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

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: spacing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spacing: ";
    rosidl_generator_traits::value_to_yaml(msg.spacing, out);
    out << "\n";
  }

  // member: terrain_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "terrain_height: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain_height, out);
    out << "\n";
  }

  // member: current_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_height: ";
    rosidl_generator_traits::value_to_yaml(msg.current_height, out);
    out << "\n";
  }

  // member: pending
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pending: ";
    rosidl_generator_traits::value_to_yaml(msg.pending, out);
    out << "\n";
  }

  // member: loaded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loaded: ";
    rosidl_generator_traits::value_to_yaml(msg.loaded, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TerrainReport & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::TerrainReport & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::TerrainReport & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::TerrainReport>()
{
  return "mavros_msgs::msg::TerrainReport";
}

template<>
inline const char * name<mavros_msgs::msg::TerrainReport>()
{
  return "mavros_msgs/msg/TerrainReport";
}

template<>
struct has_fixed_size<mavros_msgs::msg::TerrainReport>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::TerrainReport>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::TerrainReport>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__TERRAIN_REPORT__TRAITS_HPP_
