// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/WorkMapElement.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/work_map_element__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'polygon'
#include "common_interface/msg/detail/work_map_point__traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorkMapElement & msg,
  std::ostream & out)
{
  out << "{";
  // member: element_id
  {
    out << "element_id: ";
    rosidl_generator_traits::value_to_yaml(msg.element_id, out);
    out << ", ";
  }

  // member: contour_type
  {
    out << "contour_type: ";
    rosidl_generator_traits::value_to_yaml(msg.contour_type, out);
    out << ", ";
  }

  // member: polygon
  {
    if (msg.polygon.size() == 0) {
      out << "polygon: []";
    } else {
      out << "polygon: [";
      size_t pending_items = msg.polygon.size();
      for (auto item : msg.polygon) {
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
  const WorkMapElement & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: element_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "element_id: ";
    rosidl_generator_traits::value_to_yaml(msg.element_id, out);
    out << "\n";
  }

  // member: contour_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contour_type: ";
    rosidl_generator_traits::value_to_yaml(msg.contour_type, out);
    out << "\n";
  }

  // member: polygon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.polygon.size() == 0) {
      out << "polygon: []\n";
    } else {
      out << "polygon:\n";
      for (auto item : msg.polygon) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorkMapElement & msg, bool use_flow_style = false)
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
  const common_interface::msg::WorkMapElement & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::WorkMapElement & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::WorkMapElement>()
{
  return "common_interface::msg::WorkMapElement";
}

template<>
inline const char * name<common_interface::msg::WorkMapElement>()
{
  return "common_interface/msg/WorkMapElement";
}

template<>
struct has_fixed_size<common_interface::msg::WorkMapElement>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::WorkMapElement>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::WorkMapElement>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__TRAITS_HPP_
