// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/PlayTuneV2.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/play_tune_v2__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PlayTuneV2 & msg,
  std::ostream & out)
{
  out << "{";
  // member: format
  {
    out << "format: ";
    rosidl_generator_traits::value_to_yaml(msg.format, out);
    out << ", ";
  }

  // member: tune
  {
    out << "tune: ";
    rosidl_generator_traits::value_to_yaml(msg.tune, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayTuneV2 & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: format
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "format: ";
    rosidl_generator_traits::value_to_yaml(msg.format, out);
    out << "\n";
  }

  // member: tune
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tune: ";
    rosidl_generator_traits::value_to_yaml(msg.tune, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayTuneV2 & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::PlayTuneV2 & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::PlayTuneV2 & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::PlayTuneV2>()
{
  return "mavros_msgs::msg::PlayTuneV2";
}

template<>
inline const char * name<mavros_msgs::msg::PlayTuneV2>()
{
  return "mavros_msgs/msg/PlayTuneV2";
}

template<>
struct has_fixed_size<mavros_msgs::msg::PlayTuneV2>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::PlayTuneV2>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::PlayTuneV2>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__TRAITS_HPP_
