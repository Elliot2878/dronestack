// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/GimbalGetInformation.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_GET_INFORMATION__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_GET_INFORMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/gimbal_get_information__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalGetInformation_Request>()
{
  return ::mavros_msgs::srv::GimbalGetInformation_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalGetInformation_Response_result
{
public:
  explicit Init_GimbalGetInformation_Response_result(::mavros_msgs::srv::GimbalGetInformation_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalGetInformation_Response result(::mavros_msgs::srv::GimbalGetInformation_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalGetInformation_Response msg_;
};

class Init_GimbalGetInformation_Response_success
{
public:
  Init_GimbalGetInformation_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalGetInformation_Response_result success(::mavros_msgs::srv::GimbalGetInformation_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GimbalGetInformation_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalGetInformation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalGetInformation_Response>()
{
  return mavros_msgs::srv::builder::Init_GimbalGetInformation_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_GET_INFORMATION__BUILDER_HPP_
