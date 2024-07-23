// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/GimbalManagerCameraTrack.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/gimbal_manager_camera_track__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerCameraTrack_Request_bottom_right_y
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_bottom_right_y(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request bottom_right_y(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_bottom_right_y_type arg)
  {
    msg_.bottom_right_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_bottom_right_x
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_bottom_right_x(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerCameraTrack_Request_bottom_right_y bottom_right_x(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_bottom_right_x_type arg)
  {
    msg_.bottom_right_x = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_bottom_right_y(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_top_left_y
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_top_left_y(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerCameraTrack_Request_bottom_right_x top_left_y(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_top_left_y_type arg)
  {
    msg_.top_left_y = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_bottom_right_x(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_top_left_x
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_top_left_x(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerCameraTrack_Request_top_left_y top_left_x(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_top_left_x_type arg)
  {
    msg_.top_left_x = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_top_left_y(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_radius
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_radius(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerCameraTrack_Request_top_left_x radius(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_top_left_x(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_y
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_y(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerCameraTrack_Request_radius y(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_radius(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_x
{
public:
  explicit Init_GimbalManagerCameraTrack_Request_x(::mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerCameraTrack_Request_y x(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_y(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

class Init_GimbalManagerCameraTrack_Request_mode
{
public:
  Init_GimbalManagerCameraTrack_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerCameraTrack_Request_x mode(::mavros_msgs::srv::GimbalManagerCameraTrack_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_GimbalManagerCameraTrack_Request_x(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerCameraTrack_Request>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerCameraTrack_Request_mode();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerCameraTrack_Response_result
{
public:
  explicit Init_GimbalManagerCameraTrack_Response_result(::mavros_msgs::srv::GimbalManagerCameraTrack_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Response result(::mavros_msgs::srv::GimbalManagerCameraTrack_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Response msg_;
};

class Init_GimbalManagerCameraTrack_Response_success
{
public:
  Init_GimbalManagerCameraTrack_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerCameraTrack_Response_result success(::mavros_msgs::srv::GimbalManagerCameraTrack_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GimbalManagerCameraTrack_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerCameraTrack_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerCameraTrack_Response>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerCameraTrack_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__BUILDER_HPP_
