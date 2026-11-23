// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/RemoteCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/remote_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_RemoteCtrl_cutter_height
{
public:
  explicit Init_RemoteCtrl_cutter_height(::common_interface::msg::RemoteCtrl & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::RemoteCtrl cutter_height(::common_interface::msg::RemoteCtrl::_cutter_height_type arg)
  {
    msg_.cutter_height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::RemoteCtrl msg_;
};

class Init_RemoteCtrl_cutter_rpm
{
public:
  explicit Init_RemoteCtrl_cutter_rpm(::common_interface::msg::RemoteCtrl & msg)
  : msg_(msg)
  {}
  Init_RemoteCtrl_cutter_height cutter_rpm(::common_interface::msg::RemoteCtrl::_cutter_rpm_type arg)
  {
    msg_.cutter_rpm = std::move(arg);
    return Init_RemoteCtrl_cutter_height(msg_);
  }

private:
  ::common_interface::msg::RemoteCtrl msg_;
};

class Init_RemoteCtrl_angular
{
public:
  explicit Init_RemoteCtrl_angular(::common_interface::msg::RemoteCtrl & msg)
  : msg_(msg)
  {}
  Init_RemoteCtrl_cutter_rpm angular(::common_interface::msg::RemoteCtrl::_angular_type arg)
  {
    msg_.angular = std::move(arg);
    return Init_RemoteCtrl_cutter_rpm(msg_);
  }

private:
  ::common_interface::msg::RemoteCtrl msg_;
};

class Init_RemoteCtrl_linear
{
public:
  Init_RemoteCtrl_linear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RemoteCtrl_angular linear(::common_interface::msg::RemoteCtrl::_linear_type arg)
  {
    msg_.linear = std::move(arg);
    return Init_RemoteCtrl_angular(msg_);
  }

private:
  ::common_interface::msg::RemoteCtrl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::RemoteCtrl>()
{
  return common_interface::msg::builder::Init_RemoteCtrl_linear();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__BUILDER_HPP_
