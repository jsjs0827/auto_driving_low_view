// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:srv/ExitProductionTestMode.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__EXIT_PRODUCTION_TEST_MODE__TRAITS_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__EXIT_PRODUCTION_TEST_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/srv/detail/exit_production_test_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExitProductionTestMode_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExitProductionTestMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExitProductionTestMode_Request & msg, bool use_flow_style = false)
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
  const common_interface::srv::ExitProductionTestMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::ExitProductionTestMode_Request & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::ExitProductionTestMode_Request>()
{
  return "common_interface::srv::ExitProductionTestMode_Request";
}

template<>
inline const char * name<common_interface::srv::ExitProductionTestMode_Request>()
{
  return "common_interface/srv/ExitProductionTestMode_Request";
}

template<>
struct has_fixed_size<common_interface::srv::ExitProductionTestMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::srv::ExitProductionTestMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::srv::ExitProductionTestMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExitProductionTestMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: error_codes
  {
    if (msg.error_codes.size() == 0) {
      out << "error_codes: []";
    } else {
      out << "error_codes: [";
      size_t pending_items = msg.error_codes.size();
      for (auto item : msg.error_codes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: error_details
  {
    if (msg.error_details.size() == 0) {
      out << "error_details: []";
    } else {
      out << "error_details: [";
      size_t pending_items = msg.error_details.size();
      for (auto item : msg.error_details) {
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
  const ExitProductionTestMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: error_codes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.error_codes.size() == 0) {
      out << "error_codes: []\n";
    } else {
      out << "error_codes:\n";
      for (auto item : msg.error_codes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: error_details
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.error_details.size() == 0) {
      out << "error_details: []\n";
    } else {
      out << "error_details:\n";
      for (auto item : msg.error_details) {
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

inline std::string to_yaml(const ExitProductionTestMode_Response & msg, bool use_flow_style = false)
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
  const common_interface::srv::ExitProductionTestMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::ExitProductionTestMode_Response & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::ExitProductionTestMode_Response>()
{
  return "common_interface::srv::ExitProductionTestMode_Response";
}

template<>
inline const char * name<common_interface::srv::ExitProductionTestMode_Response>()
{
  return "common_interface/srv/ExitProductionTestMode_Response";
}

template<>
struct has_fixed_size<common_interface::srv::ExitProductionTestMode_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::ExitProductionTestMode_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::ExitProductionTestMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common_interface::srv::ExitProductionTestMode>()
{
  return "common_interface::srv::ExitProductionTestMode";
}

template<>
inline const char * name<common_interface::srv::ExitProductionTestMode>()
{
  return "common_interface/srv/ExitProductionTestMode";
}

template<>
struct has_fixed_size<common_interface::srv::ExitProductionTestMode>
  : std::integral_constant<
    bool,
    has_fixed_size<common_interface::srv::ExitProductionTestMode_Request>::value &&
    has_fixed_size<common_interface::srv::ExitProductionTestMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<common_interface::srv::ExitProductionTestMode>
  : std::integral_constant<
    bool,
    has_bounded_size<common_interface::srv::ExitProductionTestMode_Request>::value &&
    has_bounded_size<common_interface::srv::ExitProductionTestMode_Response>::value
  >
{
};

template<>
struct is_service<common_interface::srv::ExitProductionTestMode>
  : std::true_type
{
};

template<>
struct is_service_request<common_interface::srv::ExitProductionTestMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<common_interface::srv::ExitProductionTestMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__SRV__DETAIL__EXIT_PRODUCTION_TEST_MODE__TRAITS_HPP_
