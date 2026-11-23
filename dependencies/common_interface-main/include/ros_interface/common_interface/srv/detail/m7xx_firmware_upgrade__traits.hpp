// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:srv/M7xxFirmwareUpgrade.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__M7XX_FIRMWARE_UPGRADE__TRAITS_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__M7XX_FIRMWARE_UPGRADE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/srv/detail/m7xx_firmware_upgrade__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const M7xxFirmwareUpgrade_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: package_path
  {
    out << "package_path: ";
    rosidl_generator_traits::value_to_yaml(msg.package_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const M7xxFirmwareUpgrade_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: package_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "package_path: ";
    rosidl_generator_traits::value_to_yaml(msg.package_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const M7xxFirmwareUpgrade_Request & msg, bool use_flow_style = false)
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
  const common_interface::srv::M7xxFirmwareUpgrade_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::M7xxFirmwareUpgrade_Request & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::M7xxFirmwareUpgrade_Request>()
{
  return "common_interface::srv::M7xxFirmwareUpgrade_Request";
}

template<>
inline const char * name<common_interface::srv::M7xxFirmwareUpgrade_Request>()
{
  return "common_interface/srv/M7xxFirmwareUpgrade_Request";
}

template<>
struct has_fixed_size<common_interface::srv::M7xxFirmwareUpgrade_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::M7xxFirmwareUpgrade_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::M7xxFirmwareUpgrade_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const M7xxFirmwareUpgrade_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
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
  const M7xxFirmwareUpgrade_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
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

inline std::string to_yaml(const M7xxFirmwareUpgrade_Response & msg, bool use_flow_style = false)
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
  const common_interface::srv::M7xxFirmwareUpgrade_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::M7xxFirmwareUpgrade_Response & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::M7xxFirmwareUpgrade_Response>()
{
  return "common_interface::srv::M7xxFirmwareUpgrade_Response";
}

template<>
inline const char * name<common_interface::srv::M7xxFirmwareUpgrade_Response>()
{
  return "common_interface/srv/M7xxFirmwareUpgrade_Response";
}

template<>
struct has_fixed_size<common_interface::srv::M7xxFirmwareUpgrade_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::M7xxFirmwareUpgrade_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::M7xxFirmwareUpgrade_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common_interface::srv::M7xxFirmwareUpgrade>()
{
  return "common_interface::srv::M7xxFirmwareUpgrade";
}

template<>
inline const char * name<common_interface::srv::M7xxFirmwareUpgrade>()
{
  return "common_interface/srv/M7xxFirmwareUpgrade";
}

template<>
struct has_fixed_size<common_interface::srv::M7xxFirmwareUpgrade>
  : std::integral_constant<
    bool,
    has_fixed_size<common_interface::srv::M7xxFirmwareUpgrade_Request>::value &&
    has_fixed_size<common_interface::srv::M7xxFirmwareUpgrade_Response>::value
  >
{
};

template<>
struct has_bounded_size<common_interface::srv::M7xxFirmwareUpgrade>
  : std::integral_constant<
    bool,
    has_bounded_size<common_interface::srv::M7xxFirmwareUpgrade_Request>::value &&
    has_bounded_size<common_interface::srv::M7xxFirmwareUpgrade_Response>::value
  >
{
};

template<>
struct is_service<common_interface::srv::M7xxFirmwareUpgrade>
  : std::true_type
{
};

template<>
struct is_service_request<common_interface::srv::M7xxFirmwareUpgrade_Request>
  : std::true_type
{
};

template<>
struct is_service_response<common_interface::srv::M7xxFirmwareUpgrade_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__SRV__DETAIL__M7XX_FIRMWARE_UPGRADE__TRAITS_HPP_
