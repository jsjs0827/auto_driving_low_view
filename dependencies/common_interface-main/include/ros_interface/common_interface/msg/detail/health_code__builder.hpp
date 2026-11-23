// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/HealthCode.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__HEALTH_CODE__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__HEALTH_CODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/health_code__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_HealthCode_status
{
public:
  explicit Init_HealthCode_status(::common_interface::msg::HealthCode & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::HealthCode status(::common_interface::msg::HealthCode::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::HealthCode msg_;
};

class Init_HealthCode_code
{
public:
  Init_HealthCode_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HealthCode_status code(::common_interface::msg::HealthCode::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_HealthCode_status(msg_);
  }

private:
  ::common_interface::msg::HealthCode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::HealthCode>()
{
  return common_interface::msg::builder::Init_HealthCode_code();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__HEALTH_CODE__BUILDER_HPP_
