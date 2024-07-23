// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/VehicleInfoGet.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/vehicle_info_get__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VehicleInfoGet_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: sysid
  {
    out << "sysid: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid, out);
    out << ", ";
  }

  // member: compid
  {
    out << "compid: ";
    rosidl_generator_traits::value_to_yaml(msg.compid, out);
    out << ", ";
  }

  // member: get_all
  {
    out << "get_all: ";
    rosidl_generator_traits::value_to_yaml(msg.get_all, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleInfoGet_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sysid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sysid: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid, out);
    out << "\n";
  }

  // member: compid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compid: ";
    rosidl_generator_traits::value_to_yaml(msg.compid, out);
    out << "\n";
  }

  // member: get_all
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "get_all: ";
    rosidl_generator_traits::value_to_yaml(msg.get_all, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleInfoGet_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::VehicleInfoGet_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::VehicleInfoGet_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::VehicleInfoGet_Request>()
{
  return "mavros_msgs::srv::VehicleInfoGet_Request";
}

template<>
inline const char * name<mavros_msgs::srv::VehicleInfoGet_Request>()
{
  return "mavros_msgs/srv/VehicleInfoGet_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::VehicleInfoGet_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::VehicleInfoGet_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::VehicleInfoGet_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'vehicles'
#include "mavros_msgs/msg/detail/vehicle_info__traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VehicleInfoGet_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: vehicles
  {
    if (msg.vehicles.size() == 0) {
      out << "vehicles: []";
    } else {
      out << "vehicles: [";
      size_t pending_items = msg.vehicles.size();
      for (auto item : msg.vehicles) {
        to_flow_style_yaml(item, out);
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
  const VehicleInfoGet_Response & msg,
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

  // member: vehicles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vehicles.size() == 0) {
      out << "vehicles: []\n";
    } else {
      out << "vehicles:\n";
      for (auto item : msg.vehicles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleInfoGet_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::VehicleInfoGet_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::VehicleInfoGet_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::VehicleInfoGet_Response>()
{
  return "mavros_msgs::srv::VehicleInfoGet_Response";
}

template<>
inline const char * name<mavros_msgs::srv::VehicleInfoGet_Response>()
{
  return "mavros_msgs/srv/VehicleInfoGet_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::VehicleInfoGet_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::VehicleInfoGet_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::VehicleInfoGet_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::VehicleInfoGet>()
{
  return "mavros_msgs::srv::VehicleInfoGet";
}

template<>
inline const char * name<mavros_msgs::srv::VehicleInfoGet>()
{
  return "mavros_msgs/srv/VehicleInfoGet";
}

template<>
struct has_fixed_size<mavros_msgs::srv::VehicleInfoGet>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::VehicleInfoGet_Request>::value &&
    has_fixed_size<mavros_msgs::srv::VehicleInfoGet_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::VehicleInfoGet>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::VehicleInfoGet_Request>::value &&
    has_bounded_size<mavros_msgs::srv::VehicleInfoGet_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::VehicleInfoGet>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::VehicleInfoGet_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::VehicleInfoGet_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__TRAITS_HPP_
