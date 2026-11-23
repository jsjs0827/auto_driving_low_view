// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/M720Status.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/m720_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_M720Status_sat_num
{
public:
  explicit Init_M720Status_sat_num(::common_interface::msg::M720Status & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::M720Status sat_num(::common_interface::msg::M720Status::_sat_num_type arg)
  {
    msg_.sat_num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::M720Status msg_;
};

class Init_M720Status_gnss_status
{
public:
  explicit Init_M720Status_gnss_status(::common_interface::msg::M720Status & msg)
  : msg_(msg)
  {}
  Init_M720Status_sat_num gnss_status(::common_interface::msg::M720Status::_gnss_status_type arg)
  {
    msg_.gnss_status = std::move(arg);
    return Init_M720Status_sat_num(msg_);
  }

private:
  ::common_interface::msg::M720Status msg_;
};

class Init_M720Status_sensor_status
{
public:
  explicit Init_M720Status_sensor_status(::common_interface::msg::M720Status & msg)
  : msg_(msg)
  {}
  Init_M720Status_gnss_status sensor_status(::common_interface::msg::M720Status::_sensor_status_type arg)
  {
    msg_.sensor_status = std::move(arg);
    return Init_M720Status_gnss_status(msg_);
  }

private:
  ::common_interface::msg::M720Status msg_;
};

class Init_M720Status_fusion_status
{
public:
  explicit Init_M720Status_fusion_status(::common_interface::msg::M720Status & msg)
  : msg_(msg)
  {}
  Init_M720Status_sensor_status fusion_status(::common_interface::msg::M720Status::_fusion_status_type arg)
  {
    msg_.fusion_status = std::move(arg);
    return Init_M720Status_sensor_status(msg_);
  }

private:
  ::common_interface::msg::M720Status msg_;
};

class Init_M720Status_init_status
{
public:
  Init_M720Status_init_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_M720Status_fusion_status init_status(::common_interface::msg::M720Status::_init_status_type arg)
  {
    msg_.init_status = std::move(arg);
    return Init_M720Status_fusion_status(msg_);
  }

private:
  ::common_interface::msg::M720Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::M720Status>()
{
  return common_interface::msg::builder::Init_M720Status_init_status();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__M720_STATUS__BUILDER_HPP_
