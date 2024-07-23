// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/GimbalManagerConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/gimbal_manager_configure__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerConfigure_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: sysid_primary
  {
    out << "sysid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_primary, out);
    out << ", ";
  }

  // member: compid_primary
  {
    out << "compid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_primary, out);
    out << ", ";
  }

  // member: sysid_secondary
  {
    out << "sysid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_secondary, out);
    out << ", ";
  }

  // member: compid_secondary
  {
    out << "compid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_secondary, out);
    out << ", ";
  }

  // member: gimbal_device_id
  {
    out << "gimbal_device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_device_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalManagerConfigure_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sysid_primary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sysid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_primary, out);
    out << "\n";
  }

  // member: compid_primary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_primary, out);
    out << "\n";
  }

  // member: sysid_secondary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sysid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_secondary, out);
    out << "\n";
  }

  // member: compid_secondary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_secondary, out);
    out << "\n";
  }

  // member: gimbal_device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_device_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalManagerConfigure_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::GimbalManagerConfigure_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerConfigure_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerConfigure_Request>()
{
  return "mavros_msgs::srv::GimbalManagerConfigure_Request";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerConfigure_Request>()
{
  return "mavros_msgs/srv/GimbalManagerConfigure_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerConfigure_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerConfigure_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerConfigure_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerConfigure_Response & msg,
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
  const GimbalManagerConfigure_Response & msg,
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

inline std::string to_yaml(const GimbalManagerConfigure_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::GimbalManagerConfigure_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerConfigure_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerConfigure_Response>()
{
  return "mavros_msgs::srv::GimbalManagerConfigure_Response";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerConfigure_Response>()
{
  return "mavros_msgs/srv/GimbalManagerConfigure_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerConfigure_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerConfigure_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerConfigure_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerConfigure>()
{
  return "mavros_msgs::srv::GimbalManagerConfigure";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerConfigure>()
{
  return "mavros_msgs/srv/GimbalManagerConfigure";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerConfigure>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::GimbalManagerConfigure_Request>::value &&
    has_fixed_size<mavros_msgs::srv::GimbalManagerConfigure_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerConfigure>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::GimbalManagerConfigure_Request>::value &&
    has_bounded_size<mavros_msgs::srv::GimbalManagerConfigure_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::GimbalManagerConfigure>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::GimbalManagerConfigure_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::GimbalManagerConfigure_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CONFIGURE__TRAITS_HPP_
