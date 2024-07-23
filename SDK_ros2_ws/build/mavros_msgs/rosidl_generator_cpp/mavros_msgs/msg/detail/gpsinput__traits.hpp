// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/GPSINPUT.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GPSINPUT__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GPSINPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/gpsinput__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GPSINPUT & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: fix_type
  {
    out << "fix_type: ";
    rosidl_generator_traits::value_to_yaml(msg.fix_type, out);
    out << ", ";
  }

  // member: gps_id
  {
    out << "gps_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_id, out);
    out << ", ";
  }

  // member: ignore_flags
  {
    out << "ignore_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.ignore_flags, out);
    out << ", ";
  }

  // member: time_week_ms
  {
    out << "time_week_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.time_week_ms, out);
    out << ", ";
  }

  // member: time_week
  {
    out << "time_week: ";
    rosidl_generator_traits::value_to_yaml(msg.time_week, out);
    out << ", ";
  }

  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << ", ";
  }

  // member: alt
  {
    out << "alt: ";
    rosidl_generator_traits::value_to_yaml(msg.alt, out);
    out << ", ";
  }

  // member: hdop
  {
    out << "hdop: ";
    rosidl_generator_traits::value_to_yaml(msg.hdop, out);
    out << ", ";
  }

  // member: vdop
  {
    out << "vdop: ";
    rosidl_generator_traits::value_to_yaml(msg.vdop, out);
    out << ", ";
  }

  // member: vn
  {
    out << "vn: ";
    rosidl_generator_traits::value_to_yaml(msg.vn, out);
    out << ", ";
  }

  // member: ve
  {
    out << "ve: ";
    rosidl_generator_traits::value_to_yaml(msg.ve, out);
    out << ", ";
  }

  // member: vd
  {
    out << "vd: ";
    rosidl_generator_traits::value_to_yaml(msg.vd, out);
    out << ", ";
  }

  // member: speed_accuracy
  {
    out << "speed_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_accuracy, out);
    out << ", ";
  }

  // member: horiz_accuracy
  {
    out << "horiz_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.horiz_accuracy, out);
    out << ", ";
  }

  // member: vert_accuracy
  {
    out << "vert_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.vert_accuracy, out);
    out << ", ";
  }

  // member: satellites_visible
  {
    out << "satellites_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.satellites_visible, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GPSINPUT & msg,
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

  // member: fix_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fix_type: ";
    rosidl_generator_traits::value_to_yaml(msg.fix_type, out);
    out << "\n";
  }

  // member: gps_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_id, out);
    out << "\n";
  }

  // member: ignore_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ignore_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.ignore_flags, out);
    out << "\n";
  }

  // member: time_week_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_week_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.time_week_ms, out);
    out << "\n";
  }

  // member: time_week
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_week: ";
    rosidl_generator_traits::value_to_yaml(msg.time_week, out);
    out << "\n";
  }

  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }

  // member: alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alt: ";
    rosidl_generator_traits::value_to_yaml(msg.alt, out);
    out << "\n";
  }

  // member: hdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hdop: ";
    rosidl_generator_traits::value_to_yaml(msg.hdop, out);
    out << "\n";
  }

  // member: vdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vdop: ";
    rosidl_generator_traits::value_to_yaml(msg.vdop, out);
    out << "\n";
  }

  // member: vn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vn: ";
    rosidl_generator_traits::value_to_yaml(msg.vn, out);
    out << "\n";
  }

  // member: ve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ve: ";
    rosidl_generator_traits::value_to_yaml(msg.ve, out);
    out << "\n";
  }

  // member: vd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vd: ";
    rosidl_generator_traits::value_to_yaml(msg.vd, out);
    out << "\n";
  }

  // member: speed_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_accuracy, out);
    out << "\n";
  }

  // member: horiz_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "horiz_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.horiz_accuracy, out);
    out << "\n";
  }

  // member: vert_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vert_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.vert_accuracy, out);
    out << "\n";
  }

  // member: satellites_visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "satellites_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.satellites_visible, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GPSINPUT & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::GPSINPUT & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::GPSINPUT & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::GPSINPUT>()
{
  return "mavros_msgs::msg::GPSINPUT";
}

template<>
inline const char * name<mavros_msgs::msg::GPSINPUT>()
{
  return "mavros_msgs/msg/GPSINPUT";
}

template<>
struct has_fixed_size<mavros_msgs::msg::GPSINPUT>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::GPSINPUT>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::GPSINPUT>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__GPSINPUT__TRAITS_HPP_
