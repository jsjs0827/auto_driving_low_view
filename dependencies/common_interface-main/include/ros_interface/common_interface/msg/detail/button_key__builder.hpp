// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/ButtonKey.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/button_key__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_ButtonKey_pin
{
public:
  explicit Init_ButtonKey_pin(::common_interface::msg::ButtonKey & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::ButtonKey pin(::common_interface::msg::ButtonKey::_pin_type arg)
  {
    msg_.pin = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::ButtonKey msg_;
};

class Init_ButtonKey_cmd
{
public:
  explicit Init_ButtonKey_cmd(::common_interface::msg::ButtonKey & msg)
  : msg_(msg)
  {}
  Init_ButtonKey_pin cmd(::common_interface::msg::ButtonKey::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_ButtonKey_pin(msg_);
  }

private:
  ::common_interface::msg::ButtonKey msg_;
};

class Init_ButtonKey_time
{
public:
  Init_ButtonKey_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ButtonKey_cmd time(::common_interface::msg::ButtonKey::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_ButtonKey_cmd(msg_);
  }

private:
  ::common_interface::msg::ButtonKey msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::ButtonKey>()
{
  return common_interface::msg::builder::Init_ButtonKey_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__BUILDER_HPP_
