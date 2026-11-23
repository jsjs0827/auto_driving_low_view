// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/McuCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_CTRL__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_CTRL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/mcu_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_McuCtrl_status
{
public:
  explicit Init_McuCtrl_status(::common_interface::msg::McuCtrl & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::McuCtrl status(::common_interface::msg::McuCtrl::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::McuCtrl msg_;
};

class Init_McuCtrl_cmd
{
public:
  explicit Init_McuCtrl_cmd(::common_interface::msg::McuCtrl & msg)
  : msg_(msg)
  {}
  Init_McuCtrl_status cmd(::common_interface::msg::McuCtrl::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_McuCtrl_status(msg_);
  }

private:
  ::common_interface::msg::McuCtrl msg_;
};

class Init_McuCtrl_time
{
public:
  Init_McuCtrl_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_McuCtrl_cmd time(::common_interface::msg::McuCtrl::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_McuCtrl_cmd(msg_);
  }

private:
  ::common_interface::msg::McuCtrl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::McuCtrl>()
{
  return common_interface::msg::builder::Init_McuCtrl_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_CTRL__BUILDER_HPP_
