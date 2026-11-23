// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/M720Status.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/m720_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const M720Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: init_status
  {
    out << "init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.init_status, out);
    out << ", ";
  }

  // member: fusion_status
  {
    out << "fusion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_status, out);
    out << ", ";
  }

  // member: sensor_status
  {
    out << "sensor_status: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_status, out);
    out << ", ";
  }

  // member: gnss_status
  {
    out << "gnss_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_status, out);
    out << ", ";
  }

  // member: sat_num
  {
    out << "sat_num: ";
    rosidl_generator_traits::value_to_yaml(msg.sat_num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const M720Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: init_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.init_status, out);
    out << "\n";
  }

  // member: fusion_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_status, out);
    out << "\n";
  }

  // member: sensor_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_status: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_status, out);
    out << "\n";
  }

  // member: gnss_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_status, out);
    out << "\n";
  }

  // member: sat_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sat_num: ";
    rosidl_generator_traits::value_to_yaml(msg.sat_num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const M720Status & msg, bool use_flow_style = false)
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
  const common_interface::msg::M720Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::M720Status & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::M720Status>()
{
  return "common_interface::msg::M720Status";
}

template<>
inline const char * name<common_interface::msg::M720Status>()
{
  return "common_interface/msg/M720Status";
}

template<>
struct has_fixed_size<common_interface::msg::M720Status>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::M720Status>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::M720Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__TRAITS_HPP_
