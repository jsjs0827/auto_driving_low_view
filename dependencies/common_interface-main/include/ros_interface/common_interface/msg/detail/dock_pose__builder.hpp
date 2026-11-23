// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/DockPose.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__DOCK_POSE__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__DOCK_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/dock_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_DockPose_yaw
{
public:
  explicit Init_DockPose_yaw(::common_interface::msg::DockPose & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::DockPose yaw(::common_interface::msg::DockPose::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::DockPose msg_;
};

class Init_DockPose_y
{
public:
  explicit Init_DockPose_y(::common_interface::msg::DockPose & msg)
  : msg_(msg)
  {}
  Init_DockPose_yaw y(::common_interface::msg::DockPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DockPose_yaw(msg_);
  }

private:
  ::common_interface::msg::DockPose msg_;
};

class Init_DockPose_x
{
public:
  explicit Init_DockPose_x(::common_interface::msg::DockPose & msg)
  : msg_(msg)
  {}
  Init_DockPose_y x(::common_interface::msg::DockPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DockPose_y(msg_);
  }

private:
  ::common_interface::msg::DockPose msg_;
};

class Init_DockPose_header
{
public:
  Init_DockPose_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DockPose_x header(::common_interface::msg::DockPose::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DockPose_x(msg_);
  }

private:
  ::common_interface::msg::DockPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::DockPose>()
{
  return common_interface::msg::builder::Init_DockPose_header();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__DOCK_POSE__BUILDER_HPP_
