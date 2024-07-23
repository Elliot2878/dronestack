// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/ParamPush.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_PUSH__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_PUSH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/param_push__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ParamPush_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamPush_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamPush_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::ParamPush_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::ParamPush_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::ParamPush_Request>()
{
  return "mavros_msgs::srv::ParamPush_Request";
}

template<>
inline const char * name<mavros_msgs::srv::ParamPush_Request>()
{
  return "mavros_msgs/srv/ParamPush_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::ParamPush_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::ParamPush_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::ParamPush_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ParamPush_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: param_transfered
  {
    out << "param_transfered: ";
    rosidl_generator_traits::value_to_yaml(msg.param_transfered, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamPush_Response & msg,
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

  // member: param_transfered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param_transfered: ";
    rosidl_generator_traits::value_to_yaml(msg.param_transfered, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamPush_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::ParamPush_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::ParamPush_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::ParamPush_Response>()
{
  return "mavros_msgs::srv::ParamPush_Response";
}

template<>
inline const char * name<mavros_msgs::srv::ParamPush_Response>()
{
  return "mavros_msgs/srv/ParamPush_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::ParamPush_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::ParamPush_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::ParamPush_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::ParamPush>()
{
  return "mavros_msgs::srv::ParamPush";
}

template<>
inline const char * name<mavros_msgs::srv::ParamPush>()
{
  return "mavros_msgs/srv/ParamPush";
}

template<>
struct has_fixed_size<mavros_msgs::srv::ParamPush>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::ParamPush_Request>::value &&
    has_fixed_size<mavros_msgs::srv::ParamPush_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::ParamPush>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::ParamPush_Request>::value &&
    has_bounded_size<mavros_msgs::srv::ParamPush_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::ParamPush>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::ParamPush_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::ParamPush_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_PUSH__TRAITS_HPP_
