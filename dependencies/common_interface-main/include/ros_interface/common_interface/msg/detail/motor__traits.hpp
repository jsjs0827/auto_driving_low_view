// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/Motor.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOTOR__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/motor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Motor & msg,
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

  // member: lwheel_current
  {
    out << "lwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.lwheel_current, out);
    out << ", ";
  }

  // member: rwheel_current
  {
    out << "rwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.rwheel_current, out);
    out << ", ";
  }

  // member: cutter_current
  {
    out << "cutter_current: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_current, out);
    out << ", ";
  }

  // member: lwheel_code_value
  {
    out << "lwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lwheel_code_value, out);
    out << ", ";
  }

  // member: rwheel_code_value
  {
    out << "rwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rwheel_code_value, out);
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
  const Motor & msg,
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

  // member: lwheel_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.lwheel_current, out);
    out << "\n";
  }

  // member: rwheel_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rwheel_current: ";
    rosidl_generator_traits::value_to_yaml(msg.rwheel_current, out);
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

  // member: lwheel_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.lwheel_code_value, out);
    out << "\n";
  }

  // member: rwheel_code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rwheel_code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rwheel_code_value, out);
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

inline std::string to_yaml(const Motor & msg, bool use_flow_style = false)
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
  const common_interface::msg::Motor & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::Motor & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::Motor>()
{
  return "common_interface::msg::Motor";
}

template<>
inline const char * name<common_interface::msg::Motor>()
{
  return "common_interface/msg/Motor";
}

template<>
struct has_fixed_size<common_interface::msg::Motor>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::Motor>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::Motor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOTOR__TRAITS_HPP_
