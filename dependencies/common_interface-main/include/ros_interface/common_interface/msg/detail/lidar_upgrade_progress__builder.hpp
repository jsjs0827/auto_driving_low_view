// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/LidarUpgradeProgress.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__LIDAR_UPGRADE_PROGRESS__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__LIDAR_UPGRADE_PROGRESS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/lidar_upgrade_progress__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_LidarUpgradeProgress_message
{
public:
  explicit Init_LidarUpgradeProgress_message(::common_interface::msg::LidarUpgradeProgress & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::LidarUpgradeProgress message(::common_interface::msg::LidarUpgradeProgress::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::LidarUpgradeProgress msg_;
};

class Init_LidarUpgradeProgress_result_code
{
public:
  explicit Init_LidarUpgradeProgress_result_code(::common_interface::msg::LidarUpgradeProgress & msg)
  : msg_(msg)
  {}
  Init_LidarUpgradeProgress_message result_code(::common_interface::msg::LidarUpgradeProgress::_result_code_type arg)
  {
    msg_.result_code = std::move(arg);
    return Init_LidarUpgradeProgress_message(msg_);
  }

private:
  ::common_interface::msg::LidarUpgradeProgress msg_;
};

class Init_LidarUpgradeProgress_stage
{
public:
  explicit Init_LidarUpgradeProgress_stage(::common_interface::msg::LidarUpgradeProgress & msg)
  : msg_(msg)
  {}
  Init_LidarUpgradeProgress_result_code stage(::common_interface::msg::LidarUpgradeProgress::_stage_type arg)
  {
    msg_.stage = std::move(arg);
    return Init_LidarUpgradeProgress_result_code(msg_);
  }

private:
  ::common_interface::msg::LidarUpgradeProgress msg_;
};

class Init_LidarUpgradeProgress_percent
{
public:
  Init_LidarUpgradeProgress_percent()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarUpgradeProgress_stage percent(::common_interface::msg::LidarUpgradeProgress::_percent_type arg)
  {
    msg_.percent = std::move(arg);
    return Init_LidarUpgradeProgress_stage(msg_);
  }

private:
  ::common_interface::msg::LidarUpgradeProgress msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::LidarUpgradeProgress>()
{
  return common_interface::msg::builder::Init_LidarUpgradeProgress_percent();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__LIDAR_UPGRADE_PROGRESS__BUILDER_HPP_
