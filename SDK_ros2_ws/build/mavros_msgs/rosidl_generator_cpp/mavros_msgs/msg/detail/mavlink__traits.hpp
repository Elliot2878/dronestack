// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/Mavlink.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAVLINK__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAVLINK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/mavlink__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Mavlink & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: framing_status
  {
    out << "framing_status: ";
    rosidl_generator_traits::value_to_yaml(msg.framing_status, out);
    out << ", ";
  }

  // member: magic
  {
    out << "magic: ";
    rosidl_generator_traits::value_to_yaml(msg.magic, out);
    out << ", ";
  }

  // member: len
  {
    out << "len: ";
    rosidl_generator_traits::value_to_yaml(msg.len, out);
    out << ", ";
  }

  // member: incompat_flags
  {
    out << "incompat_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.incompat_flags, out);
    out << ", ";
  }

  // member: compat_flags
  {
    out << "compat_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.compat_flags, out);
    out << ", ";
  }

  // member: seq
  {
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
    out << ", ";
  }

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

  // member: msgid
  {
    out << "msgid: ";
    rosidl_generator_traits::value_to_yaml(msg.msgid, out);
    out << ", ";
  }

  // member: checksum
  {
    out << "checksum: ";
    rosidl_generator_traits::value_to_yaml(msg.checksum, out);
    out << ", ";
  }

  // member: payload64
  {
    if (msg.payload64.size() == 0) {
      out << "payload64: []";
    } else {
      out << "payload64: [";
      size_t pending_items = msg.payload64.size();
      for (auto item : msg.payload64) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: signature
  {
    if (msg.signature.size() == 0) {
      out << "signature: []";
    } else {
      out << "signature: [";
      size_t pending_items = msg.signature.size();
      for (auto item : msg.signature) {
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
  const Mavlink & msg,
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

  // member: framing_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "framing_status: ";
    rosidl_generator_traits::value_to_yaml(msg.framing_status, out);
    out << "\n";
  }

  // member: magic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magic: ";
    rosidl_generator_traits::value_to_yaml(msg.magic, out);
    out << "\n";
  }

  // member: len
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "len: ";
    rosidl_generator_traits::value_to_yaml(msg.len, out);
    out << "\n";
  }

  // member: incompat_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "incompat_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.incompat_flags, out);
    out << "\n";
  }

  // member: compat_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compat_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.compat_flags, out);
    out << "\n";
  }

  // member: seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
    out << "\n";
  }

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

  // member: msgid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msgid: ";
    rosidl_generator_traits::value_to_yaml(msg.msgid, out);
    out << "\n";
  }

  // member: checksum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "checksum: ";
    rosidl_generator_traits::value_to_yaml(msg.checksum, out);
    out << "\n";
  }

  // member: payload64
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.payload64.size() == 0) {
      out << "payload64: []\n";
    } else {
      out << "payload64:\n";
      for (auto item : msg.payload64) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: signature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.signature.size() == 0) {
      out << "signature: []\n";
    } else {
      out << "signature:\n";
      for (auto item : msg.signature) {
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

inline std::string to_yaml(const Mavlink & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::Mavlink & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::Mavlink & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::Mavlink>()
{
  return "mavros_msgs::msg::Mavlink";
}

template<>
inline const char * name<mavros_msgs::msg::Mavlink>()
{
  return "mavros_msgs/msg/Mavlink";
}

template<>
struct has_fixed_size<mavros_msgs::msg::Mavlink>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::Mavlink>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::Mavlink>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__MAVLINK__TRAITS_HPP_
