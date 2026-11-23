// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/GetNaviStatus.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__GET_NAVI_STATUS__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__GET_NAVI_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/get_navi_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_GetNaviStatus_Request_cmd
{
public:
  Init_GetNaviStatus_Request_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::GetNaviStatus_Request cmd(::common_interface::srv::GetNaviStatus_Request::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::GetNaviStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::GetNaviStatus_Request>()
{
  return common_interface::srv::builder::Init_GetNaviStatus_Request_cmd();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_GetNaviStatus_Response_result
{
public:
  Init_GetNaviStatus_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::GetNaviStatus_Response result(::common_interface::srv::GetNaviStatus_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::GetNaviStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::GetNaviStatus_Response>()
{
  return common_interface::srv::builder::Init_GetNaviStatus_Response_result();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__GET_NAVI_STATUS__BUILDER_HPP_
