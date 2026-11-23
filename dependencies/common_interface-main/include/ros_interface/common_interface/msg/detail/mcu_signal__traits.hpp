// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/McuSignal.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/mcu_signal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const McuSignal & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: collision_signal
  {
    out << "collision_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_signal, out);
    out << ", ";
  }

  // member: suspend_signal
  {
    out << "suspend_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.suspend_signal, out);
    out << ", ";
  }

  // member: emergency_stop_o
  {
    out << "emergency_stop_o: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop_o, out);
    out << ", ";
  }

  // member: emergency_stop_h
  {
    out << "emergency_stop_h: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop_h, out);
    out << ", ";
  }

  // member: rain_signal
  {
    out << "rain_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.rain_signal, out);
    out << ", ";
  }

  // member: charging
  {
    out << "charging: ";
    rosidl_generator_traits::value_to_yaml(msg.charging, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const McuSignal & msg,
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

  // member: collision_signal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_signal, out);
    out << "\n";
  }

  // member: suspend_signal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "suspend_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.suspend_signal, out);
    out << "\n";
  }

  // member: emergency_stop_o
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop_o: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop_o, out);
    out << "\n";
  }

  // member: emergency_stop_h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop_h: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop_h, out);
    out << "\n";
  }

  // member: rain_signal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rain_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.rain_signal, out);
    out << "\n";
  }

  // member: charging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging: ";
    rosidl_generator_traits::value_to_yaml(msg.charging, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const McuSignal & msg, bool use_flow_style = false)
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
  const common_interface::msg::McuSignal & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::McuSignal & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::McuSignal>()
{
  return "common_interface::msg::McuSignal";
}

template<>
inline const char * name<common_interface::msg::McuSignal>()
{
  return "common_interface/msg/McuSignal";
}

template<>
struct has_fixed_size<common_interface::msg::McuSignal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::McuSignal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::McuSignal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_SIGNAL__TRAITS_HPP_
