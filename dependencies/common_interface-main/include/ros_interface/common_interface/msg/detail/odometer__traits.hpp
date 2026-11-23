// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__ODOMETER__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__ODOMETER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/odometer__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Odometer & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: l_wheel
  {
    out << "l_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.l_wheel, out);
    out << ", ";
  }

  // member: r_wheel
  {
    out << "r_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.r_wheel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Odometer & msg,
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

  // member: l_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.l_wheel, out);
    out << "\n";
  }

  // member: r_wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.r_wheel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Odometer & msg, bool use_flow_style = false)
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
  const common_interface::msg::Odometer & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::Odometer & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::Odometer>()
{
  return "common_interface::msg::Odometer";
}

template<>
inline const char * name<common_interface::msg::Odometer>()
{
  return "common_interface/msg/Odometer";
}

template<>
struct has_fixed_size<common_interface::msg::Odometer>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::Odometer>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::Odometer>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__ODOMETER__TRAITS_HPP_
