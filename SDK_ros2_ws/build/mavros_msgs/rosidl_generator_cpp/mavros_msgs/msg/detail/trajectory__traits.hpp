// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'point_1'
// Member 'point_2'
// Member 'point_3'
// Member 'point_4'
// Member 'point_5'
#include "mavros_msgs/msg/detail/position_target__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Trajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: point_1
  {
    out << "point_1: ";
    to_flow_style_yaml(msg.point_1, out);
    out << ", ";
  }

  // member: point_2
  {
    out << "point_2: ";
    to_flow_style_yaml(msg.point_2, out);
    out << ", ";
  }

  // member: point_3
  {
    out << "point_3: ";
    to_flow_style_yaml(msg.point_3, out);
    out << ", ";
  }

  // member: point_4
  {
    out << "point_4: ";
    to_flow_style_yaml(msg.point_4, out);
    out << ", ";
  }

  // member: point_5
  {
    out << "point_5: ";
    to_flow_style_yaml(msg.point_5, out);
    out << ", ";
  }

  // member: point_valid
  {
    if (msg.point_valid.size() == 0) {
      out << "point_valid: []";
    } else {
      out << "point_valid: [";
      size_t pending_items = msg.point_valid.size();
      for (auto item : msg.point_valid) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: command
  {
    if (msg.command.size() == 0) {
      out << "command: []";
    } else {
      out << "command: [";
      size_t pending_items = msg.command.size();
      for (auto item : msg.command) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: time_horizon
  {
    if (msg.time_horizon.size() == 0) {
      out << "time_horizon: []";
    } else {
      out << "time_horizon: [";
      size_t pending_items = msg.time_horizon.size();
      for (auto item : msg.time_horizon) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Trajectory & msg,
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

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: point_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_1:\n";
    to_block_style_yaml(msg.point_1, out, indentation + 2);
  }

  // member: point_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_2:\n";
    to_block_style_yaml(msg.point_2, out, indentation + 2);
  }

  // member: point_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_3:\n";
    to_block_style_yaml(msg.point_3, out, indentation + 2);
  }

  // member: point_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_4:\n";
    to_block_style_yaml(msg.point_4, out, indentation + 2);
  }

  // member: point_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_5:\n";
    to_block_style_yaml(msg.point_5, out, indentation + 2);
  }

  // member: point_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.point_valid.size() == 0) {
      out << "point_valid: []\n";
    } else {
      out << "point_valid:\n";
      for (auto item : msg.point_valid) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.command.size() == 0) {
      out << "command: []\n";
    } else {
      out << "command:\n";
      for (auto item : msg.command) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: time_horizon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.time_horizon.size() == 0) {
      out << "time_horizon: []\n";
    } else {
      out << "time_horizon:\n";
      for (auto item : msg.time_horizon) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Trajectory & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::Trajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::Trajectory & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::Trajectory>()
{
  return "mavros_msgs::msg::Trajectory";
}

template<>
inline const char * name<mavros_msgs::msg::Trajectory>()
{
  return "mavros_msgs/msg/Trajectory";
}

template<>
struct has_fixed_size<mavros_msgs::msg::Trajectory>
  : std::integral_constant<bool, has_fixed_size<mavros_msgs::msg::PositionTarget>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::Trajectory>
  : std::integral_constant<bool, has_bounded_size<mavros_msgs::msg::PositionTarget>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::Trajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__TRAJECTORY__TRAITS_HPP_
