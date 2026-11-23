// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/ButtonKey.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/button_key__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const ButtonKey & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << ", ";
  }

  // member: pin
  {
    out << "pin: ";
    rosidl_generator_traits::value_to_yaml(msg.pin, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ButtonKey & msg,
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

  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << "\n";
  }

  // member: pin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pin: ";
    rosidl_generator_traits::value_to_yaml(msg.pin, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ButtonKey & msg, bool use_flow_style = false)
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
  const common_interface::msg::ButtonKey & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::ButtonKey & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::ButtonKey>()
{
  return "common_interface::msg::ButtonKey";
}

template<>
inline const char * name<common_interface::msg::ButtonKey>()
{
  return "common_interface/msg/ButtonKey";
}

template<>
struct has_fixed_size<common_interface::msg::ButtonKey>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::ButtonKey>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::ButtonKey>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__BUTTON_KEY__TRAITS_HPP_
