// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/mower_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const MowerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MowerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MowerStatus & msg, bool use_flow_style = false)
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
  const common_interface::msg::MowerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::MowerStatus & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::MowerStatus>()
{
  return "common_interface::msg::MowerStatus";
}

template<>
inline const char * name<common_interface::msg::MowerStatus>()
{
  return "common_interface/msg/MowerStatus";
}

template<>
struct has_fixed_size<common_interface::msg::MowerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::MowerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::MowerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_
