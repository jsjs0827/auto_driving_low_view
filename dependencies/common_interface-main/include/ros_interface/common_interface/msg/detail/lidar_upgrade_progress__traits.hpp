// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/LidarUpgradeProgress.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__LIDAR_UPGRADE_PROGRESS__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__LIDAR_UPGRADE_PROGRESS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/lidar_upgrade_progress__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const LidarUpgradeProgress & msg,
  std::ostream & out)
{
  out << "{";
  // member: percent
  {
    out << "percent: ";
    rosidl_generator_traits::value_to_yaml(msg.percent, out);
    out << ", ";
  }

  // member: stage
  {
    out << "stage: ";
    rosidl_generator_traits::value_to_yaml(msg.stage, out);
    out << ", ";
  }

  // member: result_code
  {
    out << "result_code: ";
    rosidl_generator_traits::value_to_yaml(msg.result_code, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LidarUpgradeProgress & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "percent: ";
    rosidl_generator_traits::value_to_yaml(msg.percent, out);
    out << "\n";
  }

  // member: stage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stage: ";
    rosidl_generator_traits::value_to_yaml(msg.stage, out);
    out << "\n";
  }

  // member: result_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result_code: ";
    rosidl_generator_traits::value_to_yaml(msg.result_code, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LidarUpgradeProgress & msg, bool use_flow_style = false)
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
  const common_interface::msg::LidarUpgradeProgress & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::LidarUpgradeProgress & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::LidarUpgradeProgress>()
{
  return "common_interface::msg::LidarUpgradeProgress";
}

template<>
inline const char * name<common_interface::msg::LidarUpgradeProgress>()
{
  return "common_interface/msg/LidarUpgradeProgress";
}

template<>
struct has_fixed_size<common_interface::msg::LidarUpgradeProgress>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::LidarUpgradeProgress>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::LidarUpgradeProgress>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__LIDAR_UPGRADE_PROGRESS__TRAITS_HPP_
