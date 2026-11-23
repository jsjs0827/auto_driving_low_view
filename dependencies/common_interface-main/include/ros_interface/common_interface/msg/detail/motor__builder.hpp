// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/Motor.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/motor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_Motor_sequence
{
public:
  explicit Init_Motor_sequence(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::Motor sequence(::common_interface::msg::Motor::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_cutting_edge_speed
{
public:
  explicit Init_Motor_cutting_edge_speed(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_sequence cutting_edge_speed(::common_interface::msg::Motor::_cutting_edge_speed_type arg)
  {
    msg_.cutting_edge_speed = std::move(arg);
    return Init_Motor_sequence(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_adjust_status
{
public:
  explicit Init_Motor_adjust_status(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_cutting_edge_speed adjust_status(::common_interface::msg::Motor::_adjust_status_type arg)
  {
    msg_.adjust_status = std::move(arg);
    return Init_Motor_cutting_edge_speed(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_cutter_code_value
{
public:
  explicit Init_Motor_cutter_code_value(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_adjust_status cutter_code_value(::common_interface::msg::Motor::_cutter_code_value_type arg)
  {
    msg_.cutter_code_value = std::move(arg);
    return Init_Motor_adjust_status(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_rwheel_code_value
{
public:
  explicit Init_Motor_rwheel_code_value(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_cutter_code_value rwheel_code_value(::common_interface::msg::Motor::_rwheel_code_value_type arg)
  {
    msg_.rwheel_code_value = std::move(arg);
    return Init_Motor_cutter_code_value(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_lwheel_code_value
{
public:
  explicit Init_Motor_lwheel_code_value(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_rwheel_code_value lwheel_code_value(::common_interface::msg::Motor::_lwheel_code_value_type arg)
  {
    msg_.lwheel_code_value = std::move(arg);
    return Init_Motor_rwheel_code_value(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_cutter_current
{
public:
  explicit Init_Motor_cutter_current(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_lwheel_code_value cutter_current(::common_interface::msg::Motor::_cutter_current_type arg)
  {
    msg_.cutter_current = std::move(arg);
    return Init_Motor_lwheel_code_value(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_rwheel_current
{
public:
  explicit Init_Motor_rwheel_current(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_cutter_current rwheel_current(::common_interface::msg::Motor::_rwheel_current_type arg)
  {
    msg_.rwheel_current = std::move(arg);
    return Init_Motor_cutter_current(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_lwheel_current
{
public:
  explicit Init_Motor_lwheel_current(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_rwheel_current lwheel_current(::common_interface::msg::Motor::_lwheel_current_type arg)
  {
    msg_.lwheel_current = std::move(arg);
    return Init_Motor_rwheel_current(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_cutter_rpm
{
public:
  explicit Init_Motor_cutter_rpm(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_lwheel_current cutter_rpm(::common_interface::msg::Motor::_cutter_rpm_type arg)
  {
    msg_.cutter_rpm = std::move(arg);
    return Init_Motor_lwheel_current(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_right_wheel
{
public:
  explicit Init_Motor_right_wheel(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_cutter_rpm right_wheel(::common_interface::msg::Motor::_right_wheel_type arg)
  {
    msg_.right_wheel = std::move(arg);
    return Init_Motor_cutter_rpm(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_left_wheel
{
public:
  explicit Init_Motor_left_wheel(::common_interface::msg::Motor & msg)
  : msg_(msg)
  {}
  Init_Motor_right_wheel left_wheel(::common_interface::msg::Motor::_left_wheel_type arg)
  {
    msg_.left_wheel = std::move(arg);
    return Init_Motor_right_wheel(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

class Init_Motor_time
{
public:
  Init_Motor_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Motor_left_wheel time(::common_interface::msg::Motor::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_Motor_left_wheel(msg_);
  }

private:
  ::common_interface::msg::Motor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::Motor>()
{
  return common_interface::msg::builder::Init_Motor_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR__BUILDER_HPP_
