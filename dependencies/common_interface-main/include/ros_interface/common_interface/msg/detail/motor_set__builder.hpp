// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/MotorSet.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/motor_set__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_MotorSet_sequence
{
public:
  explicit Init_MotorSet_sequence(::common_interface::msg::MotorSet & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::MotorSet sequence(::common_interface::msg::MotorSet::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

class Init_MotorSet_adjust_motor
{
public:
  explicit Init_MotorSet_adjust_motor(::common_interface::msg::MotorSet & msg)
  : msg_(msg)
  {}
  Init_MotorSet_sequence adjust_motor(::common_interface::msg::MotorSet::_adjust_motor_type arg)
  {
    msg_.adjust_motor = std::move(arg);
    return Init_MotorSet_sequence(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

class Init_MotorSet_cutting_edge
{
public:
  explicit Init_MotorSet_cutting_edge(::common_interface::msg::MotorSet & msg)
  : msg_(msg)
  {}
  Init_MotorSet_adjust_motor cutting_edge(::common_interface::msg::MotorSet::_cutting_edge_type arg)
  {
    msg_.cutting_edge = std::move(arg);
    return Init_MotorSet_adjust_motor(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

class Init_MotorSet_cutter_rpm
{
public:
  explicit Init_MotorSet_cutter_rpm(::common_interface::msg::MotorSet & msg)
  : msg_(msg)
  {}
  Init_MotorSet_cutting_edge cutter_rpm(::common_interface::msg::MotorSet::_cutter_rpm_type arg)
  {
    msg_.cutter_rpm = std::move(arg);
    return Init_MotorSet_cutting_edge(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

class Init_MotorSet_right_wheel
{
public:
  explicit Init_MotorSet_right_wheel(::common_interface::msg::MotorSet & msg)
  : msg_(msg)
  {}
  Init_MotorSet_cutter_rpm right_wheel(::common_interface::msg::MotorSet::_right_wheel_type arg)
  {
    msg_.right_wheel = std::move(arg);
    return Init_MotorSet_cutter_rpm(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

class Init_MotorSet_left_wheel
{
public:
  explicit Init_MotorSet_left_wheel(::common_interface::msg::MotorSet & msg)
  : msg_(msg)
  {}
  Init_MotorSet_right_wheel left_wheel(::common_interface::msg::MotorSet::_left_wheel_type arg)
  {
    msg_.left_wheel = std::move(arg);
    return Init_MotorSet_right_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

class Init_MotorSet_time
{
public:
  Init_MotorSet_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSet_left_wheel time(::common_interface::msg::MotorSet::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_MotorSet_left_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorSet msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::MotorSet>()
{
  return common_interface::msg::builder::Init_MotorSet_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET__BUILDER_HPP_
