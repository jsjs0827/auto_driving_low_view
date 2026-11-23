// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/McuOtaStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__MCU_OTA_STATUS__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__MCU_OTA_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/mcu_ota_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_McuOtaStatus_progress
{
public:
  explicit Init_McuOtaStatus_progress(::common_interface::msg::McuOtaStatus & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::McuOtaStatus progress(::common_interface::msg::McuOtaStatus::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::McuOtaStatus msg_;
};

class Init_McuOtaStatus_status
{
public:
  explicit Init_McuOtaStatus_status(::common_interface::msg::McuOtaStatus & msg)
  : msg_(msg)
  {}
  Init_McuOtaStatus_progress status(::common_interface::msg::McuOtaStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_McuOtaStatus_progress(msg_);
  }

private:
  ::common_interface::msg::McuOtaStatus msg_;
};

class Init_McuOtaStatus_time
{
public:
  Init_McuOtaStatus_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_McuOtaStatus_status time(::common_interface::msg::McuOtaStatus::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_McuOtaStatus_status(msg_);
  }

private:
  ::common_interface::msg::McuOtaStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::McuOtaStatus>()
{
  return common_interface::msg::builder::Init_McuOtaStatus_time();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__MCU_OTA_STATUS__BUILDER_HPP_
