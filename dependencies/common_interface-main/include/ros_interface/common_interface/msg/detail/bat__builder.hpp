// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/Bat.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__BAT__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__BAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/bat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_Bat_sequence
{
public:
  explicit Init_Bat_sequence(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::Bat sequence(::common_interface::msg::Bat::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_code_value
{
public:
  explicit Init_Bat_code_value(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_sequence code_value(::common_interface::msg::Bat::_code_value_type arg)
  {
    msg_.code_value = std::move(arg);
    return Init_Bat_sequence(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_power
{
public:
  explicit Init_Bat_power(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_code_value power(::common_interface::msg::Bat::_power_type arg)
  {
    msg_.power = std::move(arg);
    return Init_Bat_code_value(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_temperature
{
public:
  explicit Init_Bat_temperature(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_power temperature(::common_interface::msg::Bat::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_Bat_power(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_single_voltage
{
public:
  explicit Init_Bat_single_voltage(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_temperature single_voltage(::common_interface::msg::Bat::_single_voltage_type arg)
  {
    msg_.single_voltage = std::move(arg);
    return Init_Bat_temperature(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_real_voltage
{
public:
  explicit Init_Bat_real_voltage(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_single_voltage real_voltage(::common_interface::msg::Bat::_real_voltage_type arg)
  {
    msg_.real_voltage = std::move(arg);
    return Init_Bat_single_voltage(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_parallel_num
{
public:
  explicit Init_Bat_parallel_num(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_real_voltage parallel_num(::common_interface::msg::Bat::_parallel_num_type arg)
  {
    msg_.parallel_num = std::move(arg);
    return Init_Bat_real_voltage(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_series_num
{
public:
  explicit Init_Bat_series_num(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_parallel_num series_num(::common_interface::msg::Bat::_series_num_type arg)
  {
    msg_.series_num = std::move(arg);
    return Init_Bat_parallel_num(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_capacity
{
public:
  explicit Init_Bat_capacity(::common_interface::msg::Bat & msg)
  : msg_(msg)
  {}
  Init_Bat_series_num capacity(::common_interface::msg::Bat::_capacity_type arg)
  {
    msg_.capacity = std::move(arg);
    return Init_Bat_series_num(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

class Init_Bat_voltage
{
public:
  Init_Bat_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Bat_capacity voltage(::common_interface::msg::Bat::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_Bat_capacity(msg_);
  }

private:
  ::common_interface::msg::Bat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::Bat>()
{
  return common_interface::msg::builder::Init_Bat_voltage();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__BAT__BUILDER_HPP_
