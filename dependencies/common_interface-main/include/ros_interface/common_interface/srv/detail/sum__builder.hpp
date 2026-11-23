// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/Sum.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__SUM__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__SUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/sum__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_Sum_Request_c
{
public:
  explicit Init_Sum_Request_c(::common_interface::srv::Sum_Request & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::Sum_Request c(::common_interface::srv::Sum_Request::_c_type arg)
  {
    msg_.c = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::Sum_Request msg_;
};

class Init_Sum_Request_b
{
public:
  explicit Init_Sum_Request_b(::common_interface::srv::Sum_Request & msg)
  : msg_(msg)
  {}
  Init_Sum_Request_c b(::common_interface::srv::Sum_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_Sum_Request_c(msg_);
  }

private:
  ::common_interface::srv::Sum_Request msg_;
};

class Init_Sum_Request_a
{
public:
  Init_Sum_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sum_Request_b a(::common_interface::srv::Sum_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Sum_Request_b(msg_);
  }

private:
  ::common_interface::srv::Sum_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::Sum_Request>()
{
  return common_interface::srv::builder::Init_Sum_Request_a();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_Sum_Response_sum
{
public:
  Init_Sum_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::Sum_Response sum(::common_interface::srv::Sum_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::Sum_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::Sum_Response>()
{
  return common_interface::srv::builder::Init_Sum_Response_sum();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__SUM__BUILDER_HPP_
