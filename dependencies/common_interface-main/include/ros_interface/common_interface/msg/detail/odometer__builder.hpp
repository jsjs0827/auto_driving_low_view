// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__ODOMETER__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__ODOMETER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/odometer__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_Odometer_r_wheel
{
public:
  explicit Init_Odometer_r_wheel(::common_interface::msg::Odometer & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::Odometer r_wheel(::common_interface::msg::Odometer::_r_wheel_type arg)
  {
    msg_.r_wheel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::Odometer msg_;
};

class Init_Odometer_l_wheel
{
public:
  explicit Init_Odometer_l_wheel(::common_interface::msg::Odometer & msg)
  : msg_(msg)
  {}
  Init_Odometer_r_wheel l_wheel(::common_interface::msg::Odometer::_l_wheel_type arg)
  {
    msg_.l_wheel = std::move(arg);
    return Init_Odometer_r_wheel(msg_);
  }

private:
  ::common_interface::msg::Odometer msg_;
};

class Init_Odometer_time
{
public:
  Init_Odometer_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Odometer_l_wheel time(::common_interface::msg::Odometer::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_Odometer_l_wheel(msg_);
  }

private:
  ::common_interface::msg::Odometer msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::Odometer>()
{
  return common_interface::msg::builder::Init_Odometer_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__ODOMETER__BUILDER_HPP_
