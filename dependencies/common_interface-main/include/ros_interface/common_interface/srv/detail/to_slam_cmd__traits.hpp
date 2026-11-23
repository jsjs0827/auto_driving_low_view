// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:srv/ToSlamCmd.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__TO_SLAM_CMD__TRAITS_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__TO_SLAM_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/srv/detail/to_slam_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ToSlamCmd_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << ", ";
  }

  // member: json_data
  {
    out << "json_data: ";
    rosidl_generator_traits::value_to_yaml(msg.json_data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ToSlamCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << "\n";
  }

  // member: json_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "json_data: ";
    rosidl_generator_traits::value_to_yaml(msg.json_data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ToSlamCmd_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace common_interface

namespace rosidl_generator_traits
{

[[deprecated("use common_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const common_interface::srv::ToSlamCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::ToSlamCmd_Request & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::ToSlamCmd_Request>()
{
  return "common_interface::srv::ToSlamCmd_Request";
}

template<>
inline const char * name<common_interface::srv::ToSlamCmd_Request>()
{
  return "common_interface/srv/ToSlamCmd_Request";
}

template<>
struct has_fixed_size<common_interface::srv::ToSlamCmd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::ToSlamCmd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::ToSlamCmd_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ToSlamCmd_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ToSlamCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ToSlamCmd_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace common_interface

namespace rosidl_generator_traits
{

[[deprecated("use common_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const common_interface::srv::ToSlamCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::ToSlamCmd_Response & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::ToSlamCmd_Response>()
{
  return "common_interface::srv::ToSlamCmd_Response";
}

template<>
inline const char * name<common_interface::srv::ToSlamCmd_Response>()
{
  return "common_interface/srv/ToSlamCmd_Response";
}

template<>
struct has_fixed_size<common_interface::srv::ToSlamCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::srv::ToSlamCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::srv::ToSlamCmd_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common_interface::srv::ToSlamCmd>()
{
  return "common_interface::srv::ToSlamCmd";
}

template<>
inline const char * name<common_interface::srv::ToSlamCmd>()
{
  return "common_interface/srv/ToSlamCmd";
}

template<>
struct has_fixed_size<common_interface::srv::ToSlamCmd>
  : std::integral_constant<
    bool,
    has_fixed_size<common_interface::srv::ToSlamCmd_Request>::value &&
    has_fixed_size<common_interface::srv::ToSlamCmd_Response>::value
  >
{
};

template<>
struct has_bounded_size<common_interface::srv::ToSlamCmd>
  : std::integral_constant<
    bool,
    has_bounded_size<common_interface::srv::ToSlamCmd_Request>::value &&
    has_bounded_size<common_interface::srv::ToSlamCmd_Response>::value
  >
{
};

template<>
struct is_service<common_interface::srv::ToSlamCmd>
  : std::true_type
{
};

template<>
struct is_service_request<common_interface::srv::ToSlamCmd_Request>
  : std::true_type
{
};

template<>
struct is_service_response<common_interface::srv::ToSlamCmd_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__SRV__DETAIL__TO_SLAM_CMD__TRAITS_HPP_
