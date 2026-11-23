// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/MotorSetNew.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET_NEW__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET_NEW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/motor_set_new__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_MotorSetNew_sequence
{
public:
  explicit Init_MotorSetNew_sequence(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::MotorSetNew sequence(::common_interface::msg::MotorSetNew::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_adjust_motor
{
public:
  explicit Init_MotorSetNew_adjust_motor(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_sequence adjust_motor(::common_interface::msg::MotorSetNew::_adjust_motor_type arg)
  {
    msg_.adjust_motor = std::move(arg);
    return Init_MotorSetNew_sequence(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_cutting_edge
{
public:
  explicit Init_MotorSetNew_cutting_edge(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_adjust_motor cutting_edge(::common_interface::msg::MotorSetNew::_cutting_edge_type arg)
  {
    msg_.cutting_edge = std::move(arg);
    return Init_MotorSetNew_adjust_motor(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_cutter_rpm
{
public:
  explicit Init_MotorSetNew_cutter_rpm(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_cutting_edge cutter_rpm(::common_interface::msg::MotorSetNew::_cutter_rpm_type arg)
  {
    msg_.cutter_rpm = std::move(arg);
    return Init_MotorSetNew_cutting_edge(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_right_angle
{
public:
  explicit Init_MotorSetNew_right_angle(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_cutter_rpm right_angle(::common_interface::msg::MotorSetNew::_right_angle_type arg)
  {
    msg_.right_angle = std::move(arg);
    return Init_MotorSetNew_cutter_rpm(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_left_angle
{
public:
  explicit Init_MotorSetNew_left_angle(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_right_angle left_angle(::common_interface::msg::MotorSetNew::_left_angle_type arg)
  {
    msg_.left_angle = std::move(arg);
    return Init_MotorSetNew_right_angle(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_right_back_wheel
{
public:
  explicit Init_MotorSetNew_right_back_wheel(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_left_angle right_back_wheel(::common_interface::msg::MotorSetNew::_right_back_wheel_type arg)
  {
    msg_.right_back_wheel = std::move(arg);
    return Init_MotorSetNew_left_angle(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_left_back_wheel
{
public:
  explicit Init_MotorSetNew_left_back_wheel(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_right_back_wheel left_back_wheel(::common_interface::msg::MotorSetNew::_left_back_wheel_type arg)
  {
    msg_.left_back_wheel = std::move(arg);
    return Init_MotorSetNew_right_back_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_right_front_wheel
{
public:
  explicit Init_MotorSetNew_right_front_wheel(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_left_back_wheel right_front_wheel(::common_interface::msg::MotorSetNew::_right_front_wheel_type arg)
  {
    msg_.right_front_wheel = std::move(arg);
    return Init_MotorSetNew_left_back_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_left_front_wheel
{
public:
  explicit Init_MotorSetNew_left_front_wheel(::common_interface::msg::MotorSetNew & msg)
  : msg_(msg)
  {}
  Init_MotorSetNew_right_front_wheel left_front_wheel(::common_interface::msg::MotorSetNew::_left_front_wheel_type arg)
  {
    msg_.left_front_wheel = std::move(arg);
    return Init_MotorSetNew_right_front_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

class Init_MotorSetNew_time
{
public:
  Init_MotorSetNew_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSetNew_left_front_wheel time(::common_interface::msg::MotorSetNew::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_MotorSetNew_left_front_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorSetNew msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::MotorSetNew>()
{
  return common_interface::msg::builder::Init_MotorSetNew_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET_NEW__BUILDER_HPP_
