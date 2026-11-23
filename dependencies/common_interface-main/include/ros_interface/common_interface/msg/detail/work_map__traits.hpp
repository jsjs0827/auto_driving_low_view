// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/WorkMap.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/work_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'zones'
#include "common_interface/msg/detail/work_map_zone__traits.hpp"
// Member 'pathways'
#include "common_interface/msg/detail/path_way__traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorkMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: map_id
  {
    out << "map_id: ";
    rosidl_generator_traits::value_to_yaml(msg.map_id, out);
    out << ", ";
  }

  // member: map_name
  {
    out << "map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.map_name, out);
    out << ", ";
  }

  // member: frame_id
  {
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << ", ";
  }

  // member: active
  {
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << ", ";
  }

  // member: zones
  {
    if (msg.zones.size() == 0) {
      out << "zones: []";
    } else {
      out << "zones: [";
      size_t pending_items = msg.zones.size();
      for (auto item : msg.zones) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pathways
  {
    if (msg.pathways.size() == 0) {
      out << "pathways: []";
    } else {
      out << "pathways: [";
      size_t pending_items = msg.pathways.size();
      for (auto item : msg.pathways) {
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
  const WorkMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_id: ";
    rosidl_generator_traits::value_to_yaml(msg.map_id, out);
    out << "\n";
  }

  // member: map_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.map_name, out);
    out << "\n";
  }

  // member: frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << "\n";
  }

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << "\n";
  }

  // member: zones
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.zones.size() == 0) {
      out << "zones: []\n";
    } else {
      out << "zones:\n";
      for (auto item : msg.zones) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: pathways
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pathways.size() == 0) {
      out << "pathways: []\n";
    } else {
      out << "pathways:\n";
      for (auto item : msg.pathways) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorkMap & msg, bool use_flow_style = false)
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
  const common_interface::msg::WorkMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::WorkMap & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::WorkMap>()
{
  return "common_interface::msg::WorkMap";
}

template<>
inline const char * name<common_interface::msg::WorkMap>()
{
  return "common_interface/msg/WorkMap";
}

template<>
struct has_fixed_size<common_interface::msg::WorkMap>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::WorkMap>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::WorkMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__TRAITS_HPP_
