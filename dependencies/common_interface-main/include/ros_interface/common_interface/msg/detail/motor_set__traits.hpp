// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/MotorSet.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/motor_set__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSet & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: left_wheel
  {
    out << "left_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.left_wheel, out);
    out << ", ";
  }

  // member: right_wheel
  {
    out << "right_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.right_wheel, out);
    out << ", ";
  }

  // member: cutter_rpm
  {
    out << "cutter_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_rpm, out);
    out << ", ";
  }

  // member: cutting_edge
  {
    out << "cutting_edge: ";
    rosidl_generator_traits::value_to_yaml(msg.cutting_edge, out);
    out << ", ";
  }

  // member: adjust_motor
  {
    out << "adjust_motor: ";
    rosidl_generator_traits::value_to_yaml(msg.adjust_motor, out);
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
  const MotorSet & msg,
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

  // member: left_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.left_wheel, out);
    out << "\n";
  }

  // member: right_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.right_wheel, out);
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

  // member: cutting_edge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutting_edge: ";
    rosidl_generator_traits::value_to_yaml(msg.cutting_edge, out);
    out << "\n";
  }

  // member: adjust_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjust_motor: ";
    rosidl_generator_traits::value_to_yaml(msg.adjust_motor, out);
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

inline std::string to_yaml(const MotorSet & msg, bool use_flow_style = false)
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
  const common_interface::msg::MotorSet & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::MotorSet & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::MotorSet>()
{
  return "common_interface::msg::MotorSet";
}

template<>
inline const char * name<common_interface::msg::MotorSet>()
{
  return "common_interface/msg/MotorSet";
}

template<>
struct has_fixed_size<common_interface::msg::MotorSet>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::MotorSet>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::MotorSet>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR_SET__TRAITS_HPP_
