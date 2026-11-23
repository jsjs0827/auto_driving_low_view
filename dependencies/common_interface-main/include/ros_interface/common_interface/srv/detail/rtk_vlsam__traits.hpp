// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:srv/RTKVlsam.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__TRAITS_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/srv/detail/rtk_vlsam__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const RTKVlsam_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << ", ";
  }

  // member: has_dock
  {
    out << "has_dock: ";
    rosidl_generator_traits::value_to_yaml(msg.has_dock, out);
    out << ", ";
  }

  // member: origin_latitude
  {
    out << "origin_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_latitude, out);
    out << ", ";
  }

  // member: origin_longitude
  {
    out << "origin_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_longitude, out);
    out << ", ";
  }

  // member: origin_altitude
  {
    out << "origin_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_altitude, out);
    out << ", ";
  }

  // member: map2enu_tx
  {
    out << "map2enu_tx: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_tx, out);
    out << ", ";
  }

  // member: map2enu_ty
  {
    out << "map2enu_ty: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_ty, out);
    out << ", ";
  }

  // member: map2enu_tz
  {
    out << "map2enu_tz: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_tz, out);
    out << ", ";
  }

  // member: map2enu_qx
  {
    out << "map2enu_qx: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qx, out);
    out << ", ";
  }

  // member: map2enu_qy
  {
    out << "map2enu_qy: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qy, out);
    out << ", ";
  }

  // member: map2enu_qz
  {
    out << "map2enu_qz: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qz, out);
    out << ", ";
  }

  // member: map2enu_qw
  {
    out << "map2enu_qw: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RTKVlsam_Request & msg,
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

  // member: has_dock
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_dock: ";
    rosidl_generator_traits::value_to_yaml(msg.has_dock, out);
    out << "\n";
  }

  // member: origin_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_latitude, out);
    out << "\n";
  }

  // member: origin_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_longitude, out);
    out << "\n";
  }

  // member: origin_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_altitude, out);
    out << "\n";
  }

  // member: map2enu_tx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_tx: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_tx, out);
    out << "\n";
  }

  // member: map2enu_ty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_ty: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_ty, out);
    out << "\n";
  }

  // member: map2enu_tz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_tz: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_tz, out);
    out << "\n";
  }

  // member: map2enu_qx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_qx: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qx, out);
    out << "\n";
  }

  // member: map2enu_qy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_qy: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qy, out);
    out << "\n";
  }

  // member: map2enu_qz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_qz: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qz, out);
    out << "\n";
  }

  // member: map2enu_qw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map2enu_qw: ";
    rosidl_generator_traits::value_to_yaml(msg.map2enu_qw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RTKVlsam_Request & msg, bool use_flow_style = false)
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
  const common_interface::srv::RTKVlsam_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::RTKVlsam_Request & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::RTKVlsam_Request>()
{
  return "common_interface::srv::RTKVlsam_Request";
}

template<>
inline const char * name<common_interface::srv::RTKVlsam_Request>()
{
  return "common_interface/srv/RTKVlsam_Request";
}

template<>
struct has_fixed_size<common_interface::srv::RTKVlsam_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::srv::RTKVlsam_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::srv::RTKVlsam_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const RTKVlsam_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
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
  const RTKVlsam_Response & msg,
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

  // member: session_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
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

inline std::string to_yaml(const RTKVlsam_Response & msg, bool use_flow_style = false)
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
  const common_interface::srv::RTKVlsam_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::RTKVlsam_Response & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::RTKVlsam_Response>()
{
  return "common_interface::srv::RTKVlsam_Response";
}

template<>
inline const char * name<common_interface::srv::RTKVlsam_Response>()
{
  return "common_interface/srv/RTKVlsam_Response";
}

template<>
struct has_fixed_size<common_interface::srv::RTKVlsam_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::RTKVlsam_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::RTKVlsam_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common_interface::srv::RTKVlsam>()
{
  return "common_interface::srv::RTKVlsam";
}

template<>
inline const char * name<common_interface::srv::RTKVlsam>()
{
  return "common_interface/srv/RTKVlsam";
}

template<>
struct has_fixed_size<common_interface::srv::RTKVlsam>
  : std::integral_constant<
    bool,
    has_fixed_size<common_interface::srv::RTKVlsam_Request>::value &&
    has_fixed_size<common_interface::srv::RTKVlsam_Response>::value
  >
{
};

template<>
struct has_bounded_size<common_interface::srv::RTKVlsam>
  : std::integral_constant<
    bool,
    has_bounded_size<common_interface::srv::RTKVlsam_Request>::value &&
    has_bounded_size<common_interface::srv::RTKVlsam_Response>::value
  >
{
};

template<>
struct is_service<common_interface::srv::RTKVlsam>
  : std::true_type
{
};

template<>
struct is_service_request<common_interface::srv::RTKVlsam_Request>
  : std::true_type
{
};

template<>
struct is_service_response<common_interface::srv::RTKVlsam_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__TRAITS_HPP_
