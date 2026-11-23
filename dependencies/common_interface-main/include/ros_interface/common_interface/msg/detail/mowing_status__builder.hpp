// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/MowingStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/mowing_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_MowingStatus_mowing_progress
{
public:
  explicit Init_MowingStatus_mowing_progress(::common_interface::msg::MowingStatus & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::MowingStatus mowing_progress(::common_interface::msg::MowingStatus::_mowing_progress_type arg)
  {
    msg_.mowing_progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::MowingStatus msg_;
};

class Init_MowingStatus_mowed_area
{
public:
  explicit Init_MowingStatus_mowed_area(::common_interface::msg::MowingStatus & msg)
  : msg_(msg)
  {}
  Init_MowingStatus_mowing_progress mowed_area(::common_interface::msg::MowingStatus::_mowed_area_type arg)
  {
    msg_.mowed_area = std::move(arg);
    return Init_MowingStatus_mowing_progress(msg_);
  }

private:
  ::common_interface::msg::MowingStatus msg_;
};

class Init_MowingStatus_total_mowing_area
{
public:
  Init_MowingStatus_total_mowing_area()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowingStatus_mowed_area total_mowing_area(::common_interface::msg::MowingStatus::_total_mowing_area_type arg)
  {
    msg_.total_mowing_area = std::move(arg);
    return Init_MowingStatus_mowed_area(msg_);
  }

private:
  ::common_interface::msg::MowingStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::MowingStatus>()
{
  return common_interface::msg::builder::Init_MowingStatus_total_mowing_area();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__BUILDER_HPP_
