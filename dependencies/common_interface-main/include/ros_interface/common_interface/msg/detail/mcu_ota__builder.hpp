// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/McuOta.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_OTA__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_OTA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/mcu_ota__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_McuOta_address
{
public:
  explicit Init_McuOta_address(::common_interface::msg::McuOta & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::McuOta address(::common_interface::msg::McuOta::_address_type arg)
  {
    msg_.address = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::McuOta msg_;
};

class Init_McuOta_cmd
{
public:
  explicit Init_McuOta_cmd(::common_interface::msg::McuOta & msg)
  : msg_(msg)
  {}
  Init_McuOta_address cmd(::common_interface::msg::McuOta::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_McuOta_address(msg_);
  }

private:
  ::common_interface::msg::McuOta msg_;
};

class Init_McuOta_time
{
public:
  Init_McuOta_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_McuOta_cmd time(::common_interface::msg::McuOta::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_McuOta_cmd(msg_);
  }

private:
  ::common_interface::msg::McuOta msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::McuOta>()
{
  return common_interface::msg::builder::Init_McuOta_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_OTA__BUILDER_HPP_
