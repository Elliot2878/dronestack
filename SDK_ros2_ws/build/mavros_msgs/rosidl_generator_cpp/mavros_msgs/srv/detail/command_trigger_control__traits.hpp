// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/CommandTriggerControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/command_trigger_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandTriggerControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: trigger_enable
  {
    out << "trigger_enable: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_enable, out);
    out << ", ";
  }

  // member: sequence_reset
  {
    out << "sequence_reset: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence_reset, out);
    out << ", ";
  }

  // member: trigger_pause
  {
    out << "trigger_pause: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_pause, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CommandTriggerControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: trigger_enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_enable: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_enable, out);
    out << "\n";
  }

  // member: sequence_reset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence_reset: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence_reset, out);
    out << "\n";
  }

  // member: trigger_pause
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trigger_pause: ";
    rosidl_generator_traits::value_to_yaml(msg.trigger_pause, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CommandTriggerControl_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::CommandTriggerControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::CommandTriggerControl_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::CommandTriggerControl_Request>()
{
  return "mavros_msgs::srv::CommandTriggerControl_Request";
}

template<>
inline const char * name<mavros_msgs::srv::CommandTriggerControl_Request>()
{
  return "mavros_msgs/srv/CommandTriggerControl_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandTriggerControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandTriggerControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::CommandTriggerControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CommandTriggerControl_Response & msg,
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
  const CommandTriggerControl_Response & msg,
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

inline std::string to_yaml(const CommandTriggerControl_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::CommandTriggerControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::CommandTriggerControl_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::CommandTriggerControl_Response>()
{
  return "mavros_msgs::srv::CommandTriggerControl_Response";
}

template<>
inline const char * name<mavros_msgs::srv::CommandTriggerControl_Response>()
{
  return "mavros_msgs/srv/CommandTriggerControl_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandTriggerControl_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandTriggerControl_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::CommandTriggerControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::CommandTriggerControl>()
{
  return "mavros_msgs::srv::CommandTriggerControl";
}

template<>
inline const char * name<mavros_msgs::srv::CommandTriggerControl>()
{
  return "mavros_msgs/srv/CommandTriggerControl";
}

template<>
struct has_fixed_size<mavros_msgs::srv::CommandTriggerControl>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::CommandTriggerControl_Request>::value &&
    has_fixed_size<mavros_msgs::srv::CommandTriggerControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::CommandTriggerControl>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::CommandTriggerControl_Request>::value &&
    has_bounded_size<mavros_msgs::srv::CommandTriggerControl_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::CommandTriggerControl>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::CommandTriggerControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::CommandTriggerControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__TRAITS_HPP_
