// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/CommandLong.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_LONG__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_LONG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/command_long__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandLong_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: broadcast
  {
    out << "broadcast: ";
    rosidl_generator_traits::value_to_yaml(msg.broadcast, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: confirmation
  {
    out << "confirmation: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmation, out);
    out << ", ";
  }

  // member: param1
  {
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << ", ";
  }

  // member: param2
  {
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << ", ";
  }

  // member: param3
  {
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << ", ";
  }

  // member: param4
  {
    out << "param4: ";
    rosidl_generator_traits::value_to_yaml(msg.param4, out);
    out << ", ";
  }

  // member: param5
  {
    out << "param5: ";
    rosidl_generator_traits::value_to_yaml(msg.param5, out);
    out << ", ";
  }

  // member: param6
  {
    out << "param6: ";
    rosidl_generator_traits::value_to_yaml(msg.param6, out);
    out << ", ";
  }

  // member: param7
  {
    out << "param7: ";
    rosidl_generator_traits::value_to_yaml(msg.param7, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CommandLong_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: broadcast
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "broadcast: ";
    rosidl_generator_traits::value_to_yaml(msg.broadcast, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: confirmation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirmation: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmation, out);
    out << "\n";
  }

  // member: param1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << "\n";
  }

  // member: param2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << "\n";
  }

  // member: param3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << "\n";
  }

  // member: param4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param4: ";
    rosidl_generator_traits::value_to_yaml(msg.param4, out);
    out << "\n";
  }

  // member: param5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param5: ";
    rosidl_generator_traits::value_to_yaml(msg.param5, out);
    out << "\n";
  }

  // member: param6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param6: ";
    rosidl_generator_traits::value_to_yaml(msg.param6, out);
    out << "\n";
  }

  // member: param7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param7: ";
    rosidl_generator_traits::value_to_yaml(msg.param7, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CommandLong_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::CommandLong_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::CommandLong_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::CommandLong_Request>()
{
  return "mavros_msgs::srv::CommandLong_Request";
}

template<>
inline const char * name<mavros_msgs::srv::CommandLong_Request>()
{
  return "mavros_msgs/srv/CommandLong_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandLong_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandLong_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::CommandLong_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandLong_Response & msg,
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
  const CommandLong_Response & msg,
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

inline std::string to_yaml(const CommandLong_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::CommandLong_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::CommandLong_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::CommandLong_Response>()
{
  return "mavros_msgs::srv::CommandLong_Response";
}

template<>
inline const char * name<mavros_msgs::srv::CommandLong_Response>()
{
  return "mavros_msgs/srv/CommandLong_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandLong_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandLong_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::CommandLong_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::CommandLong>()
{
  return "mavros_msgs::srv::CommandLong";
}

template<>
inline const char * name<mavros_msgs::srv::CommandLong>()
{
  return "mavros_msgs/srv/CommandLong";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandLong>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::CommandLong_Request>::value &&
    has_fixed_size<mavros_msgs::srv::CommandLong_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandLong>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::CommandLong_Request>::value &&
    has_bounded_size<mavros_msgs::srv::CommandLong_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::CommandLong>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::CommandLong_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::CommandLong_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_LONG__TRAITS_HPP_
