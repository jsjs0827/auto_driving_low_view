// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/Bat.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__BAT__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__BAT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/bat__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Bat & msg,
  std::ostream & out)
{
  out << "{";
  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: capacity
  {
    out << "capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.capacity, out);
    out << ", ";
  }

  // member: series_num
  {
    out << "series_num: ";
    rosidl_generator_traits::value_to_yaml(msg.series_num, out);
    out << ", ";
  }

  // member: parallel_num
  {
    out << "parallel_num: ";
    rosidl_generator_traits::value_to_yaml(msg.parallel_num, out);
    out << ", ";
  }

  // member: real_voltage
  {
    out << "real_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.real_voltage, out);
    out << ", ";
  }

  // member: single_voltage
  {
    out << "single_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.single_voltage, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: power
  {
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << ", ";
  }

  // member: code_value
  {
    out << "code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.code_value, out);
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
  const Bat & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: capacity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.capacity, out);
    out << "\n";
  }

  // member: series_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "series_num: ";
    rosidl_generator_traits::value_to_yaml(msg.series_num, out);
    out << "\n";
  }

  // member: parallel_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parallel_num: ";
    rosidl_generator_traits::value_to_yaml(msg.parallel_num, out);
    out << "\n";
  }

  // member: real_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.real_voltage, out);
    out << "\n";
  }

  // member: single_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "single_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.single_voltage, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << "\n";
  }

  // member: code_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code_value: ";
    rosidl_generator_traits::value_to_yaml(msg.code_value, out);
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

inline std::string to_yaml(const Bat & msg, bool use_flow_style = false)
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
  const common_interface::msg::Bat & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::Bat & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::Bat>()
{
  return "common_interface::msg::Bat";
}

template<>
inline const char * name<common_interface::msg::Bat>()
{
  return "common_interface/msg/Bat";
}

template<>
struct has_fixed_size<common_interface::msg::Bat>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::Bat>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::Bat>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__BAT__TRAITS_HPP_
