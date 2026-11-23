// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/MotorNew.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/motor_new__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorNew & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: left_front_wheel
  {
    out << "left_front_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel, out);
    out << ", ";
  }

  // member: right_front_wheel
  {
    out << "right_front_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel, out);
    out << ", ";
  }

  // member: left_back_wheel
  {
    out << "left_back_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.left_back_wheel, out);
    out << ", ";
  }

  // member: right_back_wheel
  {
    out << "right_back_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.right_back_wheel, out);
    out << ", ";
  }

  // member: left_angle
  {
    out << "left_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.left_angle, out);
    out << ", ";
  }

  // member: right_angle
  {
    out << "right_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.right_angle, out);
    out << ", ";
  }

  // member: cutter_rpm
  {
    out << "cutter_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_rpm, out);
    out << ", ";
  }

  // member: lfwheel_current
  {
    out << "lfwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.lfwheel_current, out);
    out << ", ";
  }

  // member: rfwheel_current
  {
    out << "rfwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.rfwheel_current, out);
    out << ", ";
  }

  // member: lbwheel_current
  {
    out << "lbwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.lbwheel_current, out);
    out << ", ";
  }

  // member: rbwheel_current
  {
    out << "rbwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.rbwheel_current, out);
    out << ", ";
  }

  // member: left_turn_current
  {
    out << "left_turn_current: ";
    rosidl_generator_traits::value_to_yaml(msg.left_turn_current, out);
    out << ", ";
  }

  // member: right_turn_current
  {
    out << "right_turn_current: ";
    rosidl_generator_traits::value_to_yaml(msg.right_turn_current, out);
    out << ", ";
  }

  // member: cutter_current
  {
    out << "cutter_current: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_current, out);
    out << ", ";
  }

  // member: lfwheel_code_value
  {
    out << "lfwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lfwheel_code_value, out);
    out << ", ";
  }

  // member: rfwheel_code_value
  {
    out << "rfwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rfwheel_code_value, out);
    out << ", ";
  }

  // member: lbwheel_code_value
  {
    out << "lbwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lbwheel_code_value, out);
    out << ", ";
  }

  // member: rbwheel_code_value
  {
    out << "rbwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rbwheel_code_value, out);
    out << ", ";
  }

  // member: lturn_code_value
  {
    out << "lturn_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lturn_code_value, out);
    out << ", ";
  }

  // member: rturn_code_value
  {
    out << "rturn_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rturn_code_value, out);
    out << ", ";
  }

  // member: cutter_code_value
  {
    out << "cutter_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_code_value, out);
    out << ", ";
  }

  // member: adjust_status
  {
    out << "adjust_status: ";
    rosidl_generator_traits::value_to_yaml(msg.adjust_status, out);
    out << ", ";
  }

  // member: cutting_edge_speed
  {
    out << "cutting_edge_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.cutting_edge_speed, out);
    out << ", ";
  }

  // member: sequence
  {
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorNew & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }

  // member: left_front_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_wheel, out);
    out << "\n";
  }

  // member: right_front_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_wheel, out);
    out << "\n";
  }

  // member: left_back_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_back_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.left_back_wheel, out);
    out << "\n";
  }

  // member: right_back_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_back_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.right_back_wheel, out);
    out << "\n";
  }

  // member: left_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.left_angle, out);
    out << "\n";
  }

  // member: right_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.right_angle, out);
    out << "\n";
  }

  // member: cutter_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutter_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_rpm, out);
    out << "\n";
  }

  // member: lfwheel_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lfwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.lfwheel_current, out);
    out << "\n";
  }

  // member: rfwheel_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rfwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.rfwheel_current, out);
    out << "\n";
  }

  // member: lbwheel_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lbwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.lbwheel_current, out);
    out << "\n";
  }

  // member: rbwheel_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rbwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.rbwheel_current, out);
    out << "\n";
  }

  // member: left_turn_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_turn_current: ";
    rosidl_generator_traits::value_to_yaml(msg.left_turn_current, out);
    out << "\n";
  }

  // member: right_turn_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_turn_current: ";
    rosidl_generator_traits::value_to_yaml(msg.right_turn_current, out);
    out << "\n";
  }

  // member: cutter_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutter_current: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_current, out);
    out << "\n";
  }

  // member: lfwheel_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lfwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lfwheel_code_value, out);
    out << "\n";
  }

  // member: rfwheel_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rfwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rfwheel_code_value, out);
    out << "\n";
  }

  // member: lbwheel_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lbwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lbwheel_code_value, out);
    out << "\n";
  }

  // member: rbwheel_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rbwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rbwheel_code_value, out);
    out << "\n";
  }

  // member: lturn_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lturn_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lturn_code_value, out);
    out << "\n";
  }

  // member: rturn_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rturn_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rturn_code_value, out);
    out << "\n";
  }

  // member: cutter_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutter_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_code_value, out);
    out << "\n";
  }

  // member: adjust_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjust_status: ";
    rosidl_generator_traits::value_to_yaml(msg.adjust_status, out);
    out << "\n";
  }

  // member: cutting_edge_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutting_edge_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.cutting_edge_speed, out);
    out << "\n";
  }

  // member: sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorNew & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace common_interface

namespace rosidl_generator_traits
{

[[deprecated("use common_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const common_interface::msg::MotorNew & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::MotorNew & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::MotorNew>()
{
  return "common_interface::msg::MotorNew";
}

template<>
inline const char * name<common_interface::msg::MotorNew>()
{
  return "common_interface/msg/MotorNew";
}

template<>
struct has_fixed_size<common_interface::msg::MotorNew>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::MotorNew>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::MotorNew>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_NEW__TRAITS_HPP_
