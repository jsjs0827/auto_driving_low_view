// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/RemoteCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/remote_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RemoteCtrl & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear
  {
    out << "linear: ";
    rosidl_generator_traits::value_to_yaml(msg.linear, out);
    out << ", ";
  }

  // member: angular
  {
    out << "angular: ";
    rosidl_generator_traits::value_to_yaml(msg.angular, out);
    out << ", ";
  }

  // member: cutter_rpm
  {
    out << "cutter_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_rpm, out);
    out << ", ";
  }

  // member: cutter_height
  {
    out << "cutter_height: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_height, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RemoteCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear: ";
    rosidl_generator_traits::value_to_yaml(msg.linear, out);
    out << "\n";
  }

  // member: angular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular: ";
    rosidl_generator_traits::value_to_yaml(msg.angular, out);
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

  // member: cutter_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutter_height: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter_height, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RemoteCtrl & msg, bool use_flow_style = false)
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
  const common_interface::msg::RemoteCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::RemoteCtrl & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::RemoteCtrl>()
{
  return "common_interface::msg::RemoteCtrl";
}

template<>
inline const char * name<common_interface::msg::RemoteCtrl>()
{
  return "common_interface/msg/RemoteCtrl";
}

template<>
struct has_fixed_size<common_interface::msg::RemoteCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::RemoteCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::RemoteCtrl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__REMOTE_CTRL__TRAITS_HPP_
