// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/LightCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/light_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_LightCtrl_data_show
{
public:
  explicit Init_LightCtrl_data_show(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::LightCtrl data_show(::common_interface::msg::LightCtrl::_data_show_type arg)
  {
    msg_.data_show = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_cmd
{
public:
  explicit Init_LightCtrl_cmd(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  Init_LightCtrl_data_show cmd(::common_interface::msg::LightCtrl::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_LightCtrl_data_show(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_signal_4g
{
public:
  explicit Init_LightCtrl_signal_4g(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  Init_LightCtrl_cmd signal_4g(::common_interface::msg::LightCtrl::_signal_4g_type arg)
  {
    msg_.signal_4g = std::move(arg);
    return Init_LightCtrl_cmd(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_ble
{
public:
  explicit Init_LightCtrl_ble(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  Init_LightCtrl_signal_4g ble(::common_interface::msg::LightCtrl::_ble_type arg)
  {
    msg_.ble = std::move(arg);
    return Init_LightCtrl_signal_4g(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_lock
{
public:
  explicit Init_LightCtrl_lock(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  Init_LightCtrl_ble lock(::common_interface::msg::LightCtrl::_lock_type arg)
  {
    msg_.lock = std::move(arg);
    return Init_LightCtrl_ble(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_wifi
{
public:
  explicit Init_LightCtrl_wifi(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  Init_LightCtrl_lock wifi(::common_interface::msg::LightCtrl::_wifi_type arg)
  {
    msg_.wifi = std::move(arg);
    return Init_LightCtrl_lock(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_map
{
public:
  explicit Init_LightCtrl_map(::common_interface::msg::LightCtrl & msg)
  : msg_(msg)
  {}
  Init_LightCtrl_wifi map(::common_interface::msg::LightCtrl::_map_type arg)
  {
    msg_.map = std::move(arg);
    return Init_LightCtrl_wifi(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

class Init_LightCtrl_cutter
{
public:
  Init_LightCtrl_cutter()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LightCtrl_map cutter(::common_interface::msg::LightCtrl::_cutter_type arg)
  {
    msg_.cutter = std::move(arg);
    return Init_LightCtrl_map(msg_);
  }

private:
  ::common_interface::msg::LightCtrl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::LightCtrl>()
{
  return common_interface::msg::builder::Init_LightCtrl_cutter();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__BUILDER_HPP_
