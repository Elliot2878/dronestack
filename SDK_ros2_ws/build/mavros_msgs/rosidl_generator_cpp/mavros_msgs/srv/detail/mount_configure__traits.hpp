// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/MountConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/mount_configure__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MountConfigure_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: stabilize_roll
  {
    out << "stabilize_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.stabilize_roll, out);
    out << ", ";
  }

  // member: stabilize_pitch
  {
    out << "stabilize_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.stabilize_pitch, out);
    out << ", ";
  }

  // member: stabilize_yaw
  {
    out << "stabilize_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.stabilize_yaw, out);
    out << ", ";
  }

  // member: roll_input
  {
    out << "roll_input: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_input, out);
    out << ", ";
  }

  // member: pitch_input
  {
    out << "pitch_input: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_input, out);
    out << ", ";
  }

  // member: yaw_input
  {
    out << "yaw_input: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_input, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MountConfigure_Request & msg,
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

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: stabilize_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stabilize_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.stabilize_roll, out);
    out << "\n";
  }

  // member: stabilize_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stabilize_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.stabilize_pitch, out);
    out << "\n";
  }

  // member: stabilize_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stabilize_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.stabilize_yaw, out);
    out << "\n";
  }

  // member: roll_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_input: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_input, out);
    out << "\n";
  }

  // member: pitch_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_input: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_input, out);
    out << "\n";
  }

  // member: yaw_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_input: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_input, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MountConfigure_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::srv::MountConfigure_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::MountConfigure_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::MountConfigure_Request>()
{
  return "mavros_msgs::srv::MountConfigure_Request";
}

template<>
inline const char * name<mavros_msgs::srv::MountConfigure_Request>()
{
  return "mavros_msgs/srv/MountConfigure_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::MountConfigure_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::srv::MountConfigure_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::srv::MountConfigure_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MountConfigure_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MountConfigure_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MountConfigure_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::srv::MountConfigure_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::MountConfigure_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::MountConfigure_Response>()
{
  return "mavros_msgs::srv::MountConfigure_Response";
}

template<>
inline const char * name<mavros_msgs::srv::MountConfigure_Response>()
{
  return "mavros_msgs/srv/MountConfigure_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::MountConfigure_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::MountConfigure_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::MountConfigure_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::MountConfigure>()
{
  return "mavros_msgs::srv::MountConfigure";
}

template<>
inline const char * name<mavros_msgs::srv::MountConfigure>()
{
  return "mavros_msgs/srv/MountConfigure";
}

template<>
struct has_fixed_size<mavros_msgs::srv::MountConfigure>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::MountConfigure_Request>::value &&
    has_fixed_size<mavros_msgs::srv::MountConfigure_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::MountConfigure>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::MountConfigure_Request>::value &&
    has_bounded_size<mavros_msgs::srv::MountConfigure_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::MountConfigure>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::MountConfigure_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::MountConfigure_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__TRAITS_HPP_
