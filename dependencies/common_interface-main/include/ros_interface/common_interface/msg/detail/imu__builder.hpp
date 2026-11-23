// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__IMU__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__IMU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_Imu_temperature
{
public:
  explicit Init_Imu_temperature(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::Imu temperature(::common_interface::msg::Imu::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_accel_z
{
public:
  explicit Init_Imu_accel_z(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_temperature accel_z(::common_interface::msg::Imu::_accel_z_type arg)
  {
    msg_.accel_z = std::move(arg);
    return Init_Imu_temperature(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_accel_y
{
public:
  explicit Init_Imu_accel_y(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_accel_z accel_y(::common_interface::msg::Imu::_accel_y_type arg)
  {
    msg_.accel_y = std::move(arg);
    return Init_Imu_accel_z(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_accel_x
{
public:
  explicit Init_Imu_accel_x(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_accel_y accel_x(::common_interface::msg::Imu::_accel_x_type arg)
  {
    msg_.accel_x = std::move(arg);
    return Init_Imu_accel_y(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_gyro_z
{
public:
  explicit Init_Imu_gyro_z(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_accel_x gyro_z(::common_interface::msg::Imu::_gyro_z_type arg)
  {
    msg_.gyro_z = std::move(arg);
    return Init_Imu_accel_x(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_gyro_y
{
public:
  explicit Init_Imu_gyro_y(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_gyro_z gyro_y(::common_interface::msg::Imu::_gyro_y_type arg)
  {
    msg_.gyro_y = std::move(arg);
    return Init_Imu_gyro_z(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_gyro_x
{
public:
  explicit Init_Imu_gyro_x(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_gyro_y gyro_x(::common_interface::msg::Imu::_gyro_x_type arg)
  {
    msg_.gyro_x = std::move(arg);
    return Init_Imu_gyro_y(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_yaw
{
public:
  explicit Init_Imu_yaw(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_gyro_x yaw(::common_interface::msg::Imu::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Imu_gyro_x(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_pitch
{
public:
  explicit Init_Imu_pitch(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_yaw pitch(::common_interface::msg::Imu::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Imu_yaw(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_roll
{
public:
  explicit Init_Imu_roll(::common_interface::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_pitch roll(::common_interface::msg::Imu::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Imu_pitch(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

class Init_Imu_time
{
public:
  Init_Imu_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Imu_roll time(::common_interface::msg::Imu::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_Imu_roll(msg_);
  }

private:
  ::common_interface::msg::Imu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::Imu>()
{
  return common_interface::msg::builder::Init_Imu_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__IMU__BUILDER_HPP_
