// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/PathWay.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/path_way__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'polygon'
#include "common_interface/msg/detail/work_map_point__traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const PathWay & msg,
  std::ostream & out)
{
  out << "{";
  // member: pathway_id
  {
    out << "pathway_id: ";
    rosidl_generator_traits::value_to_yaml(msg.pathway_id, out);
    out << ", ";
  }

  // member: pathway_name
  {
    out << "pathway_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pathway_name, out);
    out << ", ";
  }

  // member: parent_id
  {
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << ", ";
  }

  // member: from_zone_id
  {
    out << "from_zone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.from_zone_id, out);
    out << ", ";
  }

  // member: to_zone_id
  {
    out << "to_zone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.to_zone_id, out);
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
    out << ", ";
  }

  // member: pathway_width
  {
    out << "pathway_width: ";
    rosidl_generator_traits::value_to_yaml(msg.pathway_width, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PathWay & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pathway_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pathway_id: ";
    rosidl_generator_traits::value_to_yaml(msg.pathway_id, out);
    out << "\n";
  }

  // member: pathway_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pathway_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pathway_name, out);
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

  // member: from_zone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "from_zone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.from_zone_id, out);
    out << "\n";
  }

  // member: to_zone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "to_zone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.to_zone_id, out);
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

  // member: pathway_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pathway_width: ";
    rosidl_generator_traits::value_to_yaml(msg.pathway_width, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PathWay & msg, bool use_flow_style = false)
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
  const common_interface::msg::PathWay & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::PathWay & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::PathWay>()
{
  return "common_interface::msg::PathWay";
}

template<>
inline const char * name<common_interface::msg::PathWay>()
{
  return "common_interface/msg/PathWay";
}

template<>
struct has_fixed_size<common_interface::msg::PathWay>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::PathWay>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::PathWay>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__TRAITS_HPP_
