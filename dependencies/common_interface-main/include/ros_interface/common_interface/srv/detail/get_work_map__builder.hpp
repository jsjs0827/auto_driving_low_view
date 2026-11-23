// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/GetWorkMap.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__GET_WORK_MAP__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__GET_WORK_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/get_work_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_GetWorkMap_Request_map_id
{
public:
  explicit Init_GetWorkMap_Request_map_id(::common_interface::srv::GetWorkMap_Request & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::GetWorkMap_Request map_id(::common_interface::srv::GetWorkMap_Request::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::GetWorkMap_Request msg_;
};

class Init_GetWorkMap_Request_cmd
{
public:
  Init_GetWorkMap_Request_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetWorkMap_Request_map_id cmd(::common_interface::srv::GetWorkMap_Request::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_GetWorkMap_Request_map_id(msg_);
  }

private:
  ::common_interface::srv::GetWorkMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::GetWorkMap_Request>()
{
  return common_interface::srv::builder::Init_GetWorkMap_Request_cmd();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_GetWorkMap_Response_maps
{
public:
  explicit Init_GetWorkMap_Response_maps(::common_interface::srv::GetWorkMap_Response & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::GetWorkMap_Response maps(::common_interface::srv::GetWorkMap_Response::_maps_type arg)
  {
    msg_.maps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::GetWorkMap_Response msg_;
};

class Init_GetWorkMap_Response_result
{
public:
  Init_GetWorkMap_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetWorkMap_Response_maps result(::common_interface::srv::GetWorkMap_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_GetWorkMap_Response_maps(msg_);
  }

private:
  ::common_interface::srv::GetWorkMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::GetWorkMap_Response>()
{
  return common_interface::srv::builder::Init_GetWorkMap_Response_result();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__GET_WORK_MAP__BUILDER_HPP_
