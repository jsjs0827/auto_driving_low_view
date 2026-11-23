// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/ToNaviCmd.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__TO_NAVI_CMD__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__TO_NAVI_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/to_navi_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_ToNaviCmd_Request_json_data
{
public:
  explicit Init_ToNaviCmd_Request_json_data(::common_interface::srv::ToNaviCmd_Request & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::ToNaviCmd_Request json_data(::common_interface::srv::ToNaviCmd_Request::_json_data_type arg)
  {
    msg_.json_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::ToNaviCmd_Request msg_;
};

class Init_ToNaviCmd_Request_cmd
{
public:
  Init_ToNaviCmd_Request_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToNaviCmd_Request_json_data cmd(::common_interface::srv::ToNaviCmd_Request::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_ToNaviCmd_Request_json_data(msg_);
  }

private:
  ::common_interface::srv::ToNaviCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::ToNaviCmd_Request>()
{
  return common_interface::srv::builder::Init_ToNaviCmd_Request_cmd();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_ToNaviCmd_Response_result
{
public:
  Init_ToNaviCmd_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::ToNaviCmd_Response result(::common_interface::srv::ToNaviCmd_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::ToNaviCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::ToNaviCmd_Response>()
{
  return common_interface::srv::builder::Init_ToNaviCmd_Response_result();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__TO_NAVI_CMD__BUILDER_HPP_
