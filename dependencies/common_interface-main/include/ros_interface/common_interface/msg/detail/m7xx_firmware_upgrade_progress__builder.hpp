// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/M7xxFirmwareUpgradeProgress.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__M7XX_FIRMWARE_UPGRADE_PROGRESS__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__M7XX_FIRMWARE_UPGRADE_PROGRESS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/m7xx_firmware_upgrade_progress__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_M7xxFirmwareUpgradeProgress_message
{
public:
  explicit Init_M7xxFirmwareUpgradeProgress_message(::common_interface::msg::M7xxFirmwareUpgradeProgress & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::M7xxFirmwareUpgradeProgress message(::common_interface::msg::M7xxFirmwareUpgradeProgress::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::M7xxFirmwareUpgradeProgress msg_;
};

class Init_M7xxFirmwareUpgradeProgress_result_code
{
public:
  explicit Init_M7xxFirmwareUpgradeProgress_result_code(::common_interface::msg::M7xxFirmwareUpgradeProgress & msg)
  : msg_(msg)
  {}
  Init_M7xxFirmwareUpgradeProgress_message result_code(::common_interface::msg::M7xxFirmwareUpgradeProgress::_result_code_type arg)
  {
    msg_.result_code = std::move(arg);
    return Init_M7xxFirmwareUpgradeProgress_message(msg_);
  }

private:
  ::common_interface::msg::M7xxFirmwareUpgradeProgress msg_;
};

class Init_M7xxFirmwareUpgradeProgress_stage
{
public:
  explicit Init_M7xxFirmwareUpgradeProgress_stage(::common_interface::msg::M7xxFirmwareUpgradeProgress & msg)
  : msg_(msg)
  {}
  Init_M7xxFirmwareUpgradeProgress_result_code stage(::common_interface::msg::M7xxFirmwareUpgradeProgress::_stage_type arg)
  {
    msg_.stage = std::move(arg);
    return Init_M7xxFirmwareUpgradeProgress_result_code(msg_);
  }

private:
  ::common_interface::msg::M7xxFirmwareUpgradeProgress msg_;
};

class Init_M7xxFirmwareUpgradeProgress_percent
{
public:
  Init_M7xxFirmwareUpgradeProgress_percent()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_M7xxFirmwareUpgradeProgress_stage percent(::common_interface::msg::M7xxFirmwareUpgradeProgress::_percent_type arg)
  {
    msg_.percent = std::move(arg);
    return Init_M7xxFirmwareUpgradeProgress_stage(msg_);
  }

private:
  ::common_interface::msg::M7xxFirmwareUpgradeProgress msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::M7xxFirmwareUpgradeProgress>()
{
  return common_interface::msg::builder::Init_M7xxFirmwareUpgradeProgress_percent();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__M7XX_FIRMWARE_UPGRADE_PROGRESS__BUILDER_HPP_
