// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/PlayTuneV2.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/play_tune_v2__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_PlayTuneV2_tune
{
public:
  explicit Init_PlayTuneV2_tune(::mavros_msgs::msg::PlayTuneV2 & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::PlayTuneV2 tune(::mavros_msgs::msg::PlayTuneV2::_tune_type arg)
  {
    msg_.tune = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::PlayTuneV2 msg_;
};

class Init_PlayTuneV2_format
{
public:
  Init_PlayTuneV2_format()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlayTuneV2_tune format(::mavros_msgs::msg::PlayTuneV2::_format_type arg)
  {
    msg_.format = std::move(arg);
    return Init_PlayTuneV2_tune(msg_);
  }

private:
  ::mavros_msgs::msg::PlayTuneV2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::PlayTuneV2>()
{
  return mavros_msgs::msg::builder::Init_PlayTuneV2_format();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__PLAY_TUNE_V2__BUILDER_HPP_
