// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/McuSignal.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/mcu_signal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_McuSignal_charging
{
public:
  explicit Init_McuSignal_charging(::common_interface::msg::McuSignal & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::McuSignal charging(::common_interface::msg::McuSignal::_charging_type arg)
  {
    msg_.charging = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

class Init_McuSignal_rain_signal
{
public:
  explicit Init_McuSignal_rain_signal(::common_interface::msg::McuSignal & msg)
  : msg_(msg)
  {}
  Init_McuSignal_charging rain_signal(::common_interface::msg::McuSignal::_rain_signal_type arg)
  {
    msg_.rain_signal = std::move(arg);
    return Init_McuSignal_charging(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

class Init_McuSignal_emergency_stop_h
{
public:
  explicit Init_McuSignal_emergency_stop_h(::common_interface::msg::McuSignal & msg)
  : msg_(msg)
  {}
  Init_McuSignal_rain_signal emergency_stop_h(::common_interface::msg::McuSignal::_emergency_stop_h_type arg)
  {
    msg_.emergency_stop_h = std::move(arg);
    return Init_McuSignal_rain_signal(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

class Init_McuSignal_emergency_stop_o
{
public:
  explicit Init_McuSignal_emergency_stop_o(::common_interface::msg::McuSignal & msg)
  : msg_(msg)
  {}
  Init_McuSignal_emergency_stop_h emergency_stop_o(::common_interface::msg::McuSignal::_emergency_stop_o_type arg)
  {
    msg_.emergency_stop_o = std::move(arg);
    return Init_McuSignal_emergency_stop_h(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

class Init_McuSignal_suspend_signal
{
public:
  explicit Init_McuSignal_suspend_signal(::common_interface::msg::McuSignal & msg)
  : msg_(msg)
  {}
  Init_McuSignal_emergency_stop_o suspend_signal(::common_interface::msg::McuSignal::_suspend_signal_type arg)
  {
    msg_.suspend_signal = std::move(arg);
    return Init_McuSignal_emergency_stop_o(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

class Init_McuSignal_collision_signal
{
public:
  explicit Init_McuSignal_collision_signal(::common_interface::msg::McuSignal & msg)
  : msg_(msg)
  {}
  Init_McuSignal_suspend_signal collision_signal(::common_interface::msg::McuSignal::_collision_signal_type arg)
  {
    msg_.collision_signal = std::move(arg);
    return Init_McuSignal_suspend_signal(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

class Init_McuSignal_time
{
public:
  Init_McuSignal_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_McuSignal_collision_signal time(::common_interface::msg::McuSignal::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_McuSignal_collision_signal(msg_);
  }

private:
  ::common_interface::msg::McuSignal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::McuSignal>()
{
  return common_interface::msg::builder::Init_McuSignal_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__BUILDER_HPP_
