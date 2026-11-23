// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common_interface:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__IMU__TRAITS_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__IMU__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "common_interface/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace common_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Imu & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: gyro_x
  {
    out << "gyro_x: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_x, out);
    out << ", ";
  }

  // member: gyro_y
  {
    out << "gyro_y: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_y, out);
    out << ", ";
  }

  // member: gyro_z
  {
    out << "gyro_z: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_z, out);
    out << ", ";
  }

  // member: accel_x
  {
    out << "accel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_x, out);
    out << ", ";
  }

  // member: accel_y
  {
    out << "accel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_y, out);
    out << ", ";
  }

  // member: accel_z
  {
    out << "accel_z: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_z, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Imu & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: gyro_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_x: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_x, out);
    out << "\n";
  }

  // member: gyro_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_y: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_y, out);
    out << "\n";
  }

  // member: gyro_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_z: ";
    rosidl_generator_traits::value_to_yaml(msg.gyro_z, out);
    out << "\n";
  }

  // member: accel_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_x, out);
    out << "\n";
  }

  // member: accel_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_y, out);
    out << "\n";
  }

  // member: accel_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_z: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_z, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Imu & msg, bool use_flow_style = false)
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
  const common_interface::msg::Imu & msg,
  std::ostream & out, size_t indentation = 0)
{
  common_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use common_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const common_interface::msg::Imu & msg)
{
  return common_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<common_interface::msg::Imu>()
{
  return "common_interface::msg::Imu";
}

template<>
inline const char * name<common_interface::msg::Imu>()
{
  return "common_interface/msg/Imu";
}

template<>
struct has_fixed_size<common_interface::msg::Imu>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<common_interface::msg::Imu>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<common_interface::msg::Imu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON_INTERFACE__MSG__DETAIL__IMU__TRAITS_HPP_
