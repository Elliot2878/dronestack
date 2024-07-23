// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/estimator_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EstimatorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: attitude_status_flag
  {
    out << "attitude_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.attitude_status_flag, out);
    out << ", ";
  }

  // member: velocity_horiz_status_flag
  {
    out << "velocity_horiz_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_horiz_status_flag, out);
    out << ", ";
  }

  // member: velocity_vert_status_flag
  {
    out << "velocity_vert_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_vert_status_flag, out);
    out << ", ";
  }

  // member: pos_horiz_rel_status_flag
  {
    out << "pos_horiz_rel_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_horiz_rel_status_flag, out);
    out << ", ";
  }

  // member: pos_horiz_abs_status_flag
  {
    out << "pos_horiz_abs_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_horiz_abs_status_flag, out);
    out << ", ";
  }

  // member: pos_vert_abs_status_flag
  {
    out << "pos_vert_abs_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_vert_abs_status_flag, out);
    out << ", ";
  }

  // member: pos_vert_agl_status_flag
  {
    out << "pos_vert_agl_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_vert_agl_status_flag, out);
    out << ", ";
  }

  // member: const_pos_mode_status_flag
  {
    out << "const_pos_mode_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.const_pos_mode_status_flag, out);
    out << ", ";
  }

  // member: pred_pos_horiz_rel_status_flag
  {
    out << "pred_pos_horiz_rel_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pred_pos_horiz_rel_status_flag, out);
    out << ", ";
  }

  // member: pred_pos_horiz_abs_status_flag
  {
    out << "pred_pos_horiz_abs_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pred_pos_horiz_abs_status_flag, out);
    out << ", ";
  }

  // member: gps_glitch_status_flag
  {
    out << "gps_glitch_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_glitch_status_flag, out);
    out << ", ";
  }

  // member: accel_error_status_flag
  {
    out << "accel_error_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_error_status_flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EstimatorStatus & msg,
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

  // member: attitude_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attitude_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.attitude_status_flag, out);
    out << "\n";
  }

  // member: velocity_horiz_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_horiz_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_horiz_status_flag, out);
    out << "\n";
  }

  // member: velocity_vert_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_vert_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_vert_status_flag, out);
    out << "\n";
  }

  // member: pos_horiz_rel_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_horiz_rel_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_horiz_rel_status_flag, out);
    out << "\n";
  }

  // member: pos_horiz_abs_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_horiz_abs_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_horiz_abs_status_flag, out);
    out << "\n";
  }

  // member: pos_vert_abs_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_vert_abs_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_vert_abs_status_flag, out);
    out << "\n";
  }

  // member: pos_vert_agl_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_vert_agl_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_vert_agl_status_flag, out);
    out << "\n";
  }

  // member: const_pos_mode_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "const_pos_mode_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.const_pos_mode_status_flag, out);
    out << "\n";
  }

  // member: pred_pos_horiz_rel_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pred_pos_horiz_rel_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pred_pos_horiz_rel_status_flag, out);
    out << "\n";
  }

  // member: pred_pos_horiz_abs_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pred_pos_horiz_abs_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.pred_pos_horiz_abs_status_flag, out);
    out << "\n";
  }

  // member: gps_glitch_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_glitch_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_glitch_status_flag, out);
    out << "\n";
  }

  // member: accel_error_status_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_error_status_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_error_status_flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EstimatorStatus & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::EstimatorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::EstimatorStatus & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::EstimatorStatus>()
{
  return "mavros_msgs::msg::EstimatorStatus";
}

template<>
inline const char * name<mavros_msgs::msg::EstimatorStatus>()
{
  return "mavros_msgs/msg/EstimatorStatus";
}

template<>
struct has_fixed_size<mavros_msgs::msg::EstimatorStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::EstimatorStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::EstimatorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__ESTIMATOR_STATUS__TRAITS_HPP_
