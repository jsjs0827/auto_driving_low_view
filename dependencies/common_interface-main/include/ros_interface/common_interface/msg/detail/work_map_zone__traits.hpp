// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/WorkMapZone.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/work_map_zone__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'elements'
#include "common_interface/msg/detail/work_map_element__traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorkMapZone & msg,
  std::ostream & out)
{
  out << "{";
  // member: zone_id
  {
    out << "zone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.zone_id, out);
    out << ", ";
  }

  // member: zone_name
  {
    out << "zone_name: ";
    rosidl_generator_traits::value_to_yaml(msg.zone_name, out);
    out << ", ";
  }

  // member: parent_id
  {
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << ", ";
  }

  // member: elements
  {
    if (msg.elements.size() == 0) {
      out << "elements: []";
    } else {
      out << "elements: [";
      size_t pending_items = msg.elements.size();
      for (auto item : msg.elements) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WorkMapZone & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: zone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "zone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.zone_id, out);
    out << "\n";
  }

  // member: zone_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "zone_name: ";
    rosidl_generator_traits::value_to_yaml(msg.zone_name, out);
    out << "\n";
  }

  // member: parent_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << "\n";
  }

  // member: elements
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.elements.size() == 0) {
      out << "elements: []\n";
    } else {
      out << "elements:\n";
      for (auto item : msg.elements) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorkMapZone & msg, bool use_flow_style = false)
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
  const common_interface::msg::WorkMapZone & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::WorkMapZone & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::WorkMapZone>()
{
  return "common_interface::msg::WorkMapZone";
}

template<>
inline const char * name<common_interface::msg::WorkMapZone>()
{
  return "common_interface/msg/WorkMapZone";
}

template<>
struct has_fixed_size<common_interface::msg::WorkMapZone>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::WorkMapZone>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::WorkMapZone>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__TRAITS_HPP_
