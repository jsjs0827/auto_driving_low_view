// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/MowingStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/mowing_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const MowingStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_mowing_area
  {
    out << "total_mowing_area: ";
    rosidl_generator_traits::value_to_yaml(msg.total_mowing_area, out);
    out << ", ";
  }

  // member: mowed_area
  {
    out << "mowed_area: ";
    rosidl_generator_traits::value_to_yaml(msg.mowed_area, out);
    out << ", ";
  }

  // member: mowing_progress
  {
    out << "mowing_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.mowing_progress, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MowingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_mowing_area
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_mowing_area: ";
    rosidl_generator_traits::value_to_yaml(msg.total_mowing_area, out);
    out << "\n";
  }

  // member: mowed_area
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mowed_area: ";
    rosidl_generator_traits::value_to_yaml(msg.mowed_area, out);
    out << "\n";
  }

  // member: mowing_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mowing_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.mowing_progress, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MowingStatus & msg, bool use_flow_style = false)
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
  const common_interface::msg::MowingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::MowingStatus & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::MowingStatus>()
{
  return "common_interface::msg::MowingStatus";
}

template<>
inline const char * name<common_interface::msg::MowingStatus>()
{
  return "common_interface/msg/MowingStatus";
}

template<>
struct has_fixed_size<common_interface::msg::MowingStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::MowingStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::MowingStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__MOWING_STATUS__TRAITS_HPP_
