// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/PoseSeed.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/pose_seed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_PoseSeed_Request_seed
{
public:
  explicit Init_PoseSeed_Request_seed(::common_interface::srv::PoseSeed_Request & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::PoseSeed_Request seed(::common_interface::srv::PoseSeed_Request::_seed_type arg)
  {
    msg_.seed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Request msg_;
};

class Init_PoseSeed_Request_has_pose_seed
{
public:
  explicit Init_PoseSeed_Request_has_pose_seed(::common_interface::srv::PoseSeed_Request & msg)
  : msg_(msg)
  {}
  Init_PoseSeed_Request_seed has_pose_seed(::common_interface::srv::PoseSeed_Request::_has_pose_seed_type arg)
  {
    msg_.has_pose_seed = std::move(arg);
    return Init_PoseSeed_Request_seed(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Request msg_;
};

class Init_PoseSeed_Request_option
{
public:
  Init_PoseSeed_Request_option()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseSeed_Request_has_pose_seed option(::common_interface::srv::PoseSeed_Request::_option_type arg)
  {
    msg_.option = std::move(arg);
    return Init_PoseSeed_Request_has_pose_seed(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::PoseSeed_Request>()
{
  return common_interface::srv::builder::Init_PoseSeed_Request_option();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_PoseSeed_Response_seed
{
public:
  explicit Init_PoseSeed_Response_seed(::common_interface::srv::PoseSeed_Response & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::PoseSeed_Response seed(::common_interface::srv::PoseSeed_Response::_seed_type arg)
  {
    msg_.seed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Response msg_;
};

class Init_PoseSeed_Response_message
{
public:
  explicit Init_PoseSeed_Response_message(::common_interface::srv::PoseSeed_Response & msg)
  : msg_(msg)
  {}
  Init_PoseSeed_Response_seed message(::common_interface::srv::PoseSeed_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_PoseSeed_Response_seed(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Response msg_;
};

class Init_PoseSeed_Response_error_code
{
public:
  explicit Init_PoseSeed_Response_error_code(::common_interface::srv::PoseSeed_Response & msg)
  : msg_(msg)
  {}
  Init_PoseSeed_Response_message error_code(::common_interface::srv::PoseSeed_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_PoseSeed_Response_message(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Response msg_;
};

class Init_PoseSeed_Response_result
{
public:
  Init_PoseSeed_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseSeed_Response_error_code result(::common_interface::srv::PoseSeed_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_PoseSeed_Response_error_code(msg_);
  }

private:
  ::common_interface::srv::PoseSeed_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::PoseSeed_Response>()
{
  return common_interface::srv::builder::Init_PoseSeed_Response_result();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__POSE_SEED__BUILDER_HPP_
