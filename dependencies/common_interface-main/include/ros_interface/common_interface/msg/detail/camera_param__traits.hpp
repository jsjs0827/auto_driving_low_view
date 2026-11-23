// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/CameraParam.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/camera_param__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const CameraParam & msg,
  std::ostream & out)
{
  out << "{";
  // member: fx
  {
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << ", ";
  }

  // member: fy
  {
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << ", ";
  }

  // member: cx
  {
    out << "cx: ";
    rosidl_generator_traits::value_to_yaml(msg.cx, out);
    out << ", ";
  }

  // member: cy
  {
    out << "cy: ";
    rosidl_generator_traits::value_to_yaml(msg.cy, out);
    out << ", ";
  }

  // member: radial_distortion_k
  {
    if (msg.radial_distortion_k.size() == 0) {
      out << "radial_distortion_k: []";
    } else {
      out << "radial_distortion_k: [";
      size_t pending_items = msg.radial_distortion_k.size();
      for (auto item : msg.radial_distortion_k) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tangential_distortion_p
  {
    if (msg.tangential_distortion_p.size() == 0) {
      out << "tangential_distortion_p: []";
    } else {
      out << "tangential_distortion_p: [";
      size_t pending_items = msg.tangential_distortion_p.size();
      for (auto item : msg.tangential_distortion_p) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bxf
  {
    out << "bxf: ";
    rosidl_generator_traits::value_to_yaml(msg.bxf, out);
    out << ", ";
  }

  // member: base_line
  {
    out << "base_line: ";
    rosidl_generator_traits::value_to_yaml(msg.base_line, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: dsp_max_value
  {
    out << "dsp_max_value: ";
    rosidl_generator_traits::value_to_yaml(msg.dsp_max_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraParam & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << "\n";
  }

  // member: fy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << "\n";
  }

  // member: cx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cx: ";
    rosidl_generator_traits::value_to_yaml(msg.cx, out);
    out << "\n";
  }

  // member: cy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cy: ";
    rosidl_generator_traits::value_to_yaml(msg.cy, out);
    out << "\n";
  }

  // member: radial_distortion_k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.radial_distortion_k.size() == 0) {
      out << "radial_distortion_k: []\n";
    } else {
      out << "radial_distortion_k:\n";
      for (auto item : msg.radial_distortion_k) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tangential_distortion_p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tangential_distortion_p.size() == 0) {
      out << "tangential_distortion_p: []\n";
    } else {
      out << "tangential_distortion_p:\n";
      for (auto item : msg.tangential_distortion_p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: bxf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bxf: ";
    rosidl_generator_traits::value_to_yaml(msg.bxf, out);
    out << "\n";
  }

  // member: base_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_line: ";
    rosidl_generator_traits::value_to_yaml(msg.base_line, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: dsp_max_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dsp_max_value: ";
    rosidl_generator_traits::value_to_yaml(msg.dsp_max_value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraParam & msg, bool use_flow_style = false)
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
  const common_interface::msg::CameraParam & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::CameraParam & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::CameraParam>()
{
  return "common_interface::msg::CameraParam";
}

template<>
inline const char * name<common_interface::msg::CameraParam>()
{
  return "common_interface/msg/CameraParam";
}

template<>
struct has_fixed_size<common_interface::msg::CameraParam>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::CameraParam>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::CameraParam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__TRAITS_HPP_
