// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/MotorNew.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/motor_new__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_MotorNew_sequence
{
public:
  explicit Init_MotorNew_sequence(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::MotorNew sequence(::common_interface::msg::MotorNew::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_cutting_edge_speed
{
public:
  explicit Init_MotorNew_cutting_edge_speed(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_sequence cutting_edge_speed(::common_interface::msg::MotorNew::_cutting_edge_speed_type arg)
  {
    msg_.cutting_edge_speed = std::move(arg);
    return Init_MotorNew_sequence(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_adjust_status
{
public:
  explicit Init_MotorNew_adjust_status(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_cutting_edge_speed adjust_status(::common_interface::msg::MotorNew::_adjust_status_type arg)
  {
    msg_.adjust_status = std::move(arg);
    return Init_MotorNew_cutting_edge_speed(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_cutter_code_value
{
public:
  explicit Init_MotorNew_cutter_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_adjust_status cutter_code_value(::common_interface::msg::MotorNew::_cutter_code_value_type arg)
  {
    msg_.cutter_code_value = std::move(arg);
    return Init_MotorNew_adjust_status(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_rturn_code_value
{
public:
  explicit Init_MotorNew_rturn_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_cutter_code_value rturn_code_value(::common_interface::msg::MotorNew::_rturn_code_value_type arg)
  {
    msg_.rturn_code_value = std::move(arg);
    return Init_MotorNew_cutter_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_lturn_code_value
{
public:
  explicit Init_MotorNew_lturn_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_rturn_code_value lturn_code_value(::common_interface::msg::MotorNew::_lturn_code_value_type arg)
  {
    msg_.lturn_code_value = std::move(arg);
    return Init_MotorNew_rturn_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_rbwheel_code_value
{
public:
  explicit Init_MotorNew_rbwheel_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_lturn_code_value rbwheel_code_value(::common_interface::msg::MotorNew::_rbwheel_code_value_type arg)
  {
    msg_.rbwheel_code_value = std::move(arg);
    return Init_MotorNew_lturn_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_lbwheel_code_value
{
public:
  explicit Init_MotorNew_lbwheel_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_rbwheel_code_value lbwheel_code_value(::common_interface::msg::MotorNew::_lbwheel_code_value_type arg)
  {
    msg_.lbwheel_code_value = std::move(arg);
    return Init_MotorNew_rbwheel_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_rfwheel_code_value
{
public:
  explicit Init_MotorNew_rfwheel_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_lbwheel_code_value rfwheel_code_value(::common_interface::msg::MotorNew::_rfwheel_code_value_type arg)
  {
    msg_.rfwheel_code_value = std::move(arg);
    return Init_MotorNew_lbwheel_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_lfwheel_code_value
{
public:
  explicit Init_MotorNew_lfwheel_code_value(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_rfwheel_code_value lfwheel_code_value(::common_interface::msg::MotorNew::_lfwheel_code_value_type arg)
  {
    msg_.lfwheel_code_value = std::move(arg);
    return Init_MotorNew_rfwheel_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_cutter_current
{
public:
  explicit Init_MotorNew_cutter_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_lfwheel_code_value cutter_current(::common_interface::msg::MotorNew::_cutter_current_type arg)
  {
    msg_.cutter_current = std::move(arg);
    return Init_MotorNew_lfwheel_code_value(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_right_turn_current
{
public:
  explicit Init_MotorNew_right_turn_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_cutter_current right_turn_current(::common_interface::msg::MotorNew::_right_turn_current_type arg)
  {
    msg_.right_turn_current = std::move(arg);
    return Init_MotorNew_cutter_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_left_turn_current
{
public:
  explicit Init_MotorNew_left_turn_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_right_turn_current left_turn_current(::common_interface::msg::MotorNew::_left_turn_current_type arg)
  {
    msg_.left_turn_current = std::move(arg);
    return Init_MotorNew_right_turn_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_rbwheel_current
{
public:
  explicit Init_MotorNew_rbwheel_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_left_turn_current rbwheel_current(::common_interface::msg::MotorNew::_rbwheel_current_type arg)
  {
    msg_.rbwheel_current = std::move(arg);
    return Init_MotorNew_left_turn_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_lbwheel_current
{
public:
  explicit Init_MotorNew_lbwheel_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_rbwheel_current lbwheel_current(::common_interface::msg::MotorNew::_lbwheel_current_type arg)
  {
    msg_.lbwheel_current = std::move(arg);
    return Init_MotorNew_rbwheel_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_rfwheel_current
{
public:
  explicit Init_MotorNew_rfwheel_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_lbwheel_current rfwheel_current(::common_interface::msg::MotorNew::_rfwheel_current_type arg)
  {
    msg_.rfwheel_current = std::move(arg);
    return Init_MotorNew_lbwheel_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_lfwheel_current
{
public:
  explicit Init_MotorNew_lfwheel_current(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_rfwheel_current lfwheel_current(::common_interface::msg::MotorNew::_lfwheel_current_type arg)
  {
    msg_.lfwheel_current = std::move(arg);
    return Init_MotorNew_rfwheel_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_cutter_rpm
{
public:
  explicit Init_MotorNew_cutter_rpm(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_lfwheel_current cutter_rpm(::common_interface::msg::MotorNew::_cutter_rpm_type arg)
  {
    msg_.cutter_rpm = std::move(arg);
    return Init_MotorNew_lfwheel_current(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_right_angle
{
public:
  explicit Init_MotorNew_right_angle(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_cutter_rpm right_angle(::common_interface::msg::MotorNew::_right_angle_type arg)
  {
    msg_.right_angle = std::move(arg);
    return Init_MotorNew_cutter_rpm(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_left_angle
{
public:
  explicit Init_MotorNew_left_angle(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_right_angle left_angle(::common_interface::msg::MotorNew::_left_angle_type arg)
  {
    msg_.left_angle = std::move(arg);
    return Init_MotorNew_right_angle(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_right_back_wheel
{
public:
  explicit Init_MotorNew_right_back_wheel(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_left_angle right_back_wheel(::common_interface::msg::MotorNew::_right_back_wheel_type arg)
  {
    msg_.right_back_wheel = std::move(arg);
    return Init_MotorNew_left_angle(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_left_back_wheel
{
public:
  explicit Init_MotorNew_left_back_wheel(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_right_back_wheel left_back_wheel(::common_interface::msg::MotorNew::_left_back_wheel_type arg)
  {
    msg_.left_back_wheel = std::move(arg);
    return Init_MotorNew_right_back_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_right_front_wheel
{
public:
  explicit Init_MotorNew_right_front_wheel(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_left_back_wheel right_front_wheel(::common_interface::msg::MotorNew::_right_front_wheel_type arg)
  {
    msg_.right_front_wheel = std::move(arg);
    return Init_MotorNew_left_back_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_left_front_wheel
{
public:
  explicit Init_MotorNew_left_front_wheel(::common_interface::msg::MotorNew & msg)
  : msg_(msg)
  {}
  Init_MotorNew_right_front_wheel left_front_wheel(::common_interface::msg::MotorNew::_left_front_wheel_type arg)
  {
    msg_.left_front_wheel = std::move(arg);
    return Init_MotorNew_right_front_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

class Init_MotorNew_time
{
public:
  Init_MotorNew_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorNew_left_front_wheel time(::common_interface::msg::MotorNew::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_MotorNew_left_front_wheel(msg_);
  }

private:
  ::common_interface::msg::MotorNew msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::MotorNew>()
{
  return common_interface::msg::builder::Init_MotorNew_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__BUILDER_HPP_
