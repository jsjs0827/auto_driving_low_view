// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:srv/IotCustomCmd.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__TRAITS_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/srv/detail/iot_custom_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const IotCustomCmd_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IotCustomCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IotCustomCmd_Request & msg, bool use_flow_style = false)
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
  const common_interface::srv::IotCustomCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::IotCustomCmd_Request & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::IotCustomCmd_Request>()
{
  return "common_interface::srv::IotCustomCmd_Request";
}

template<>
inline const char * name<common_interface::srv::IotCustomCmd_Request>()
{
  return "common_interface/srv/IotCustomCmd_Request";
}

template<>
struct has_fixed_size<common_interface::srv::IotCustomCmd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::IotCustomCmd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::IotCustomCmd_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const IotCustomCmd_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: reply
  {
    out << "reply: ";
    rosidl_generator_traits::value_to_yaml(msg.reply, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IotCustomCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: reply
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reply: ";
    rosidl_generator_traits::value_to_yaml(msg.reply, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IotCustomCmd_Response & msg, bool use_flow_style = false)
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
  const common_interface::srv::IotCustomCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::IotCustomCmd_Response & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::IotCustomCmd_Response>()
{
  return "common_interface::srv::IotCustomCmd_Response";
}

template<>
inline const char * name<common_interface::srv::IotCustomCmd_Response>()
{
  return "common_interface/srv/IotCustomCmd_Response";
}

template<>
struct has_fixed_size<common_interface::srv::IotCustomCmd_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::IotCustomCmd_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::IotCustomCmd_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common_interface::srv::IotCustomCmd>()
{
  return "common_interface::srv::IotCustomCmd";
}

template<>
inline const char * name<common_interface::srv::IotCustomCmd>()
{
  return "common_interface/srv/IotCustomCmd";
}

template<>
struct has_fixed_size<common_interface::srv::IotCustomCmd>
  : std::integral_constant<
    bool,
    has_fixed_size<common_interface::srv::IotCustomCmd_Request>::value &&
    has_fixed_size<common_interface::srv::IotCustomCmd_Response>::value
  >
{
};

template<>
struct has_bounded_size<common_interface::srv::IotCustomCmd>
  : std::integral_constant<
    bool,
    has_bounded_size<common_interface::srv::IotCustomCmd_Request>::value &&
    has_bounded_size<common_interface::srv::IotCustomCmd_Response>::value
  >
{
};

template<>
struct is_service<common_interface::srv::IotCustomCmd>
  : std::true_type
{
};

template<>
struct is_service_request<common_interface::srv::IotCustomCmd_Request>
  : std::true_type
{
};

template<>
struct is_service_response<common_interface::srv::IotCustomCmd_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__TRAITS_HPP_
