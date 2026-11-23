// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:srv/PoseSeed.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__TRAITS_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/srv/detail/pose_seed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const PoseSeed_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: option
  {
    out << "option: ";
    rosidl_generator_traits::value_to_yaml(msg.option, out);
    out << ", ";
  }

  // member: has_pose_seed
  {
    out << "has_pose_seed: ";
    rosidl_generator_traits::value_to_yaml(msg.has_pose_seed, out);
    out << ", ";
  }

  // member: seed
  {
    if (msg.seed.size() == 0) {
      out << "seed: []";
    } else {
      out << "seed: [";
      size_t pending_items = msg.seed.size();
      for (auto item : msg.seed) {
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
  const PoseSeed_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: option
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "option: ";
    rosidl_generator_traits::value_to_yaml(msg.option, out);
    out << "\n";
  }

  // member: has_pose_seed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_pose_seed: ";
    rosidl_generator_traits::value_to_yaml(msg.has_pose_seed, out);
    out << "\n";
  }

  // member: seed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seed.size() == 0) {
      out << "seed: []\n";
    } else {
      out << "seed:\n";
      for (auto item : msg.seed) {
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

inline std::string to_yaml(const PoseSeed_Request & msg, bool use_flow_style = false)
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
  const common_interface::srv::PoseSeed_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::PoseSeed_Request & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::PoseSeed_Request>()
{
  return "common_interface::srv::PoseSeed_Request";
}

template<>
inline const char * name<common_interface::srv::PoseSeed_Request>()
{
  return "common_interface/srv/PoseSeed_Request";
}

template<>
struct has_fixed_size<common_interface::srv::PoseSeed_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::srv::PoseSeed_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::srv::PoseSeed_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace common_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const PoseSeed_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
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
    out << ", ";
  }

  // member: seed
  {
    if (msg.seed.size() == 0) {
      out << "seed: []";
    } else {
      out << "seed: [";
      size_t pending_items = msg.seed.size();
      for (auto item : msg.seed) {
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
  const PoseSeed_Response & msg,
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

  // member: seed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seed.size() == 0) {
      out << "seed: []\n";
    } else {
      out << "seed:\n";
      for (auto item : msg.seed) {
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

inline std::string to_yaml(const PoseSeed_Response & msg, bool use_flow_style = false)
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
  const common_interface::srv::PoseSeed_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::srv::PoseSeed_Response & msg)
{
  return common_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::srv::PoseSeed_Response>()
{
  return "common_interface::srv::PoseSeed_Response";
}

template<>
inline const char * name<common_interface::srv::PoseSeed_Response>()
{
  return "common_interface/srv/PoseSeed_Response";
}

template<>
struct has_fixed_size<common_interface::srv::PoseSeed_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<common_interface::srv::PoseSeed_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<common_interface::srv::PoseSeed_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common_interface::srv::PoseSeed>()
{
  return "common_interface::srv::PoseSeed";
}

template<>
inline const char * name<common_interface::srv::PoseSeed>()
{
  return "common_interface/srv/PoseSeed";
}

template<>
struct has_fixed_size<common_interface::srv::PoseSeed>
  : std::integral_constant<
    bool,
    has_fixed_size<common_interface::srv::PoseSeed_Request>::value &&
    has_fixed_size<common_interface::srv::PoseSeed_Response>::value
  >
{
};

template<>
struct has_bounded_size<common_interface::srv::PoseSeed>
  : std::integral_constant<
    bool,
    has_bounded_size<common_interface::srv::PoseSeed_Request>::value &&
    has_bounded_size<common_interface::srv::PoseSeed_Response>::value
  >
{
};

template<>
struct is_service<common_interface::srv::PoseSeed>
  : std::true_type
{
};

template<>
struct is_service_request<common_interface::srv::PoseSeed_Request>
  : std::true_type
{
};

template<>
struct is_service_response<common_interface::srv::PoseSeed_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__TRAITS_HPP_
