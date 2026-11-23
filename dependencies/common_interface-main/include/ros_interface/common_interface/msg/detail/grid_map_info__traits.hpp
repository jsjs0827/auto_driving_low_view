// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/GridMapInfo.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/grid_map_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const GridMapInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: rows
  {
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << ", ";
  }

  // member: cols
  {
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << ", ";
  }

  // member: resolution_mm
  {
    out << "resolution_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution_mm, out);
    out << ", ";
  }

  // member: avg_h_cm
  {
    if (msg.avg_h_cm.size() == 0) {
      out << "avg_h_cm: []";
    } else {
      out << "avg_h_cm: [";
      size_t pending_items = msg.avg_h_cm.size();
      for (auto item : msg.avg_h_cm) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: min_h_cm
  {
    if (msg.min_h_cm.size() == 0) {
      out << "min_h_cm: []";
    } else {
      out << "min_h_cm: [";
      size_t pending_items = msg.min_h_cm.size();
      for (auto item : msg.min_h_cm) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: max_h_cm
  {
    if (msg.max_h_cm.size() == 0) {
      out << "max_h_cm: []";
    } else {
      out << "max_h_cm: [";
      size_t pending_items = msg.max_h_cm.size();
      for (auto item : msg.max_h_cm) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: type
  {
    if (msg.type.size() == 0) {
      out << "type: []";
    } else {
      out << "type: [";
      size_t pending_items = msg.type.size();
      for (auto item : msg.type) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const GridMapInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: rows
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << "\n";
  }

  // member: cols
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << "\n";
  }

  // member: resolution_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resolution_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution_mm, out);
    out << "\n";
  }

  // member: avg_h_cm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.avg_h_cm.size() == 0) {
      out << "avg_h_cm: []\n";
    } else {
      out << "avg_h_cm:\n";
      for (auto item : msg.avg_h_cm) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: min_h_cm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.min_h_cm.size() == 0) {
      out << "min_h_cm: []\n";
    } else {
      out << "min_h_cm:\n";
      for (auto item : msg.min_h_cm) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: max_h_cm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.max_h_cm.size() == 0) {
      out << "max_h_cm: []\n";
    } else {
      out << "max_h_cm:\n";
      for (auto item : msg.max_h_cm) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.type.size() == 0) {
      out << "type: []\n";
    } else {
      out << "type:\n";
      for (auto item : msg.type) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GridMapInfo & msg, bool use_flow_style = false)
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
  const common_interface::msg::GridMapInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::GridMapInfo & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::GridMapInfo>()
{
  return "common_interface::msg::GridMapInfo";
}

template<>
inline const char * name<common_interface::msg::GridMapInfo>()
{
  return "common_interface/msg/GridMapInfo";
}

template<>
struct has_fixed_size<common_interface::msg::GridMapInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::msg::GridMapInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::msg::GridMapInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__TRAITS_HPP_
