// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/CommandTOLLocal.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/command_tol_local__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandTOLLocal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: min_pitch
  {
    out << "min_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.min_pitch, out);
    out << ", ";
  }

  // member: offset
  {
    out << "offset: ";
    rosidl_generator_traits::value_to_yaml(msg.offset, out);
    out << ", ";
  }

  // member: rate
  {
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CommandTOLLocal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: min_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.min_pitch, out);
    out << "\n";
  }

  // member: offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset: ";
    rosidl_generator_traits::value_to_yaml(msg.offset, out);
    out << "\n";
  }

  // member: rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
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

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CommandTOLLocal_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::CommandTOLLocal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::CommandTOLLocal_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::CommandTOLLocal_Request>()
{
  return "mavros_msgs::srv::CommandTOLLocal_Request";
}

template<>
inline const char * name<mavros_msgs::srv::CommandTOLLocal_Request>()
{
  return "mavros_msgs/srv/CommandTOLLocal_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandTOLLocal_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandTOLLocal_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<mavros_msgs::srv::CommandTOLLocal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandTOLLocal_Response & msg,
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
  const CommandTOLLocal_Response & msg,
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

inline std::string to_yaml(const CommandTOLLocal_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::CommandTOLLocal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::CommandTOLLocal_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::CommandTOLLocal_Response>()
{
  return "mavros_msgs::srv::CommandTOLLocal_Response";
}

template<>
inline const char * name<mavros_msgs::srv::CommandTOLLocal_Response>()
{
  return "mavros_msgs/srv/CommandTOLLocal_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandTOLLocal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandTOLLocal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::CommandTOLLocal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::CommandTOLLocal>()
{
  return "mavros_msgs::srv::CommandTOLLocal";
}

template<>
inline const char * name<mavros_msgs::srv::CommandTOLLocal>()
{
  return "mavros_msgs/srv/CommandTOLLocal";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandTOLLocal>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::CommandTOLLocal_Request>::value &&
    has_fixed_size<mavros_msgs::srv::CommandTOLLocal_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandTOLLocal>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::CommandTOLLocal_Request>::value &&
    has_bounded_size<mavros_msgs::srv::CommandTOLLocal_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::CommandTOLLocal>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::CommandTOLLocal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::CommandTOLLocal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__TRAITS_HPP_
