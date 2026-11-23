// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/mower_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_MowerStatus_status
{
public:
  Init_MowerStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::msg::MowerStatus status(::common_interface::msg::MowerStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::MowerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::MowerStatus>()
{
  return common_interface::msg::builder::Init_MowerStatus_status();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_
