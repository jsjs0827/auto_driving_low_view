// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/LidarUpgrade.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__LIDAR_UPGRADE__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__LIDAR_UPGRADE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/lidar_upgrade__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_LidarUpgrade_Request_package_path
{
public:
  Init_LidarUpgrade_Request_package_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::LidarUpgrade_Request package_path(::common_interface::srv::LidarUpgrade_Request::_package_path_type arg)
  {
    msg_.package_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::LidarUpgrade_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::LidarUpgrade_Request>()
{
  return common_interface::srv::builder::Init_LidarUpgrade_Request_package_path();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_LidarUpgrade_Response_message
{
public:
  explicit Init_LidarUpgrade_Response_message(::common_interface::srv::LidarUpgrade_Response & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::LidarUpgrade_Response message(::common_interface::srv::LidarUpgrade_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::LidarUpgrade_Response msg_;
};

class Init_LidarUpgrade_Response_accepted
{
public:
  Init_LidarUpgrade_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarUpgrade_Response_message accepted(::common_interface::srv::LidarUpgrade_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_LidarUpgrade_Response_message(msg_);
  }

private:
  ::common_interface::srv::LidarUpgrade_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::LidarUpgrade_Response>()
{
  return common_interface::srv::builder::Init_LidarUpgrade_Response_accepted();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__LIDAR_UPGRADE__BUILDER_HPP_
