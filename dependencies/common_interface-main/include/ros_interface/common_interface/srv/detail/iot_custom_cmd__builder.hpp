// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/IotCustomCmd.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/iot_custom_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_IotCustomCmd_Request_request
{
public:
  Init_IotCustomCmd_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::IotCustomCmd_Request request(::common_interface::srv::IotCustomCmd_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::IotCustomCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::IotCustomCmd_Request>()
{
  return common_interface::srv::builder::Init_IotCustomCmd_Request_request();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_IotCustomCmd_Response_reply
{
public:
  Init_IotCustomCmd_Response_reply()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::IotCustomCmd_Response reply(::common_interface::srv::IotCustomCmd_Response::_reply_type arg)
  {
    msg_.reply = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::IotCustomCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::IotCustomCmd_Response>()
{
  return common_interface::srv::builder::Init_IotCustomCmd_Response_reply();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__IOT_CUSTOM_CMD__BUILDER_HPP_
