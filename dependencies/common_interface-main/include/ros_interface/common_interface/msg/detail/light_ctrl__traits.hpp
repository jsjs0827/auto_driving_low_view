// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/LightCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/light_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const LightCtrl & msg,
  std::ostream & out)
{
  out << "{";
  // member: cutter
  {
    out << "cutter: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter, out);
    out << ", ";
  }

  // member: map
  {
    out << "map: ";
    rosidl_generator_traits::value_to_yaml(msg.map, out);
    out << ", ";
  }

  // member: wifi
  {
    out << "wifi: ";
    rosidl_generator_traits::value_to_yaml(msg.wifi, out);
    out << ", ";
  }

  // member: lock
  {
    out << "lock: ";
    rosidl_generator_traits::value_to_yaml(msg.lock, out);
    out << ", ";
  }

  // member: ble
  {
    out << "ble: ";
    rosidl_generator_traits::value_to_yaml(msg.ble, out);
    out << ", ";
  }

  // member: signal_4g
  {
    out << "signal_4g: ";
    rosidl_generator_traits::value_to_yaml(msg.signal_4g, out);
    out << ", ";
  }

  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << ", ";
  }

  // member: data_show
  {
    out << "data_show: ";
    rosidl_generator_traits::value_to_yaml(msg.data_show, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LightCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cutter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cutter: ";
    rosidl_generator_traits::value_to_yaml(msg.cutter, out);
    out << "\n";
  }

  // member: map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map: ";
    rosidl_generator_traits::value_to_yaml(msg.map, out);
    out << "\n";
  }

  // member: wifi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wifi: ";
    rosidl_generator_traits::value_to_yaml(msg.wifi, out);
    out << "\n";
  }

  // member: lock
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lock: ";
    rosidl_generator_traits::value_to_yaml(msg.lock, out);
    out << "\n";
  }

  // member: ble
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ble: ";
    rosidl_generator_traits::value_to_yaml(msg.ble, out);
    out << "\n";
  }

  // member: signal_4g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "signal_4g: ";
    rosidl_generator_traits::value_to_yaml(msg.signal_4g, out);
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

  // member: data_show
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_show: ";
    rosidl_generator_traits::value_to_yaml(msg.data_show, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LightCtrl & msg, bool use_flow_style = false)
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
  const common_interface::msg::LightCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::LightCtrl & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::LightCtrl>()
{
  return "common_interface::msg::LightCtrl";
}

template<>
inline const char * name<common_interface::msg::LightCtrl>()
{
  return "common_interface/msg/LightCtrl";
}

template<>
struct has_fixed_size<common_interface::msg::LightCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::LightCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::LightCtrl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__TRAITS_HPP_
