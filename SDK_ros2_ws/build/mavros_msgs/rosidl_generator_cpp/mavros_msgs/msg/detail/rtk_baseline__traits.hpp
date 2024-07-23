// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/RTKBaseline.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/rtk_baseline__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RTKBaseline & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: time_last_baseline_ms
  {
    out << "time_last_baseline_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.time_last_baseline_ms, out);
    out << ", ";
  }

  // member: rtk_receiver_id
  {
    out << "rtk_receiver_id: ";
    rosidl_generator_traits::value_to_yaml(msg.rtk_receiver_id, out);
    out << ", ";
  }

  // member: wn
  {
    out << "wn: ";
    rosidl_generator_traits::value_to_yaml(msg.wn, out);
    out << ", ";
  }

  // member: tow
  {
    out << "tow: ";
    rosidl_generator_traits::value_to_yaml(msg.tow, out);
    out << ", ";
  }

  // member: rtk_health
  {
    out << "rtk_health: ";
    rosidl_generator_traits::value_to_yaml(msg.rtk_health, out);
    out << ", ";
  }

  // member: rtk_rate
  {
    out << "rtk_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rtk_rate, out);
    out << ", ";
  }

  // member: nsats
  {
    out << "nsats: ";
    rosidl_generator_traits::value_to_yaml(msg.nsats, out);
    out << ", ";
  }

  // member: baseline_coords_type
  {
    out << "baseline_coords_type: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_coords_type, out);
    out << ", ";
  }

  // member: baseline_a_mm
  {
    out << "baseline_a_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_a_mm, out);
    out << ", ";
  }

  // member: baseline_b_mm
  {
    out << "baseline_b_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_b_mm, out);
    out << ", ";
  }

  // member: baseline_c_mm
  {
    out << "baseline_c_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_c_mm, out);
    out << ", ";
  }

  // member: accuracy
  {
    out << "accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.accuracy, out);
    out << ", ";
  }

  // member: iar_num_hypotheses
  {
    out << "iar_num_hypotheses: ";
    rosidl_generator_traits::value_to_yaml(msg.iar_num_hypotheses, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RTKBaseline & msg,
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

  // member: time_last_baseline_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_last_baseline_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.time_last_baseline_ms, out);
    out << "\n";
  }

  // member: rtk_receiver_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtk_receiver_id: ";
    rosidl_generator_traits::value_to_yaml(msg.rtk_receiver_id, out);
    out << "\n";
  }

  // member: wn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wn: ";
    rosidl_generator_traits::value_to_yaml(msg.wn, out);
    out << "\n";
  }

  // member: tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tow: ";
    rosidl_generator_traits::value_to_yaml(msg.tow, out);
    out << "\n";
  }

  // member: rtk_health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtk_health: ";
    rosidl_generator_traits::value_to_yaml(msg.rtk_health, out);
    out << "\n";
  }

  // member: rtk_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtk_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rtk_rate, out);
    out << "\n";
  }

  // member: nsats
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nsats: ";
    rosidl_generator_traits::value_to_yaml(msg.nsats, out);
    out << "\n";
  }

  // member: baseline_coords_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "baseline_coords_type: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_coords_type, out);
    out << "\n";
  }

  // member: baseline_a_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "baseline_a_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_a_mm, out);
    out << "\n";
  }

  // member: baseline_b_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "baseline_b_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_b_mm, out);
    out << "\n";
  }

  // member: baseline_c_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "baseline_c_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.baseline_c_mm, out);
    out << "\n";
  }

  // member: accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.accuracy, out);
    out << "\n";
  }

  // member: iar_num_hypotheses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iar_num_hypotheses: ";
    rosidl_generator_traits::value_to_yaml(msg.iar_num_hypotheses, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RTKBaseline & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::RTKBaseline & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::RTKBaseline & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::RTKBaseline>()
{
  return "mavros_msgs::msg::RTKBaseline";
}

template<>
inline const char * name<mavros_msgs::msg::RTKBaseline>()
{
  return "mavros_msgs/msg/RTKBaseline";
}

template<>
struct has_fixed_size<mavros_msgs::msg::RTKBaseline>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::RTKBaseline>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::RTKBaseline>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__RTK_BASELINE__TRAITS_HPP_
