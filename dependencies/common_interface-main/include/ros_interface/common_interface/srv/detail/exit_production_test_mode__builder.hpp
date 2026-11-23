// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/ExitProductionTestMode.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__EXIT_PRODUCTION_TEST_MODE__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__EXIT_PRODUCTION_TEST_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/exit_production_test_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::ExitProductionTestMode_Request>()
{
  return ::common_interface::srv::ExitProductionTestMode_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_ExitProductionTestMode_Response_error_details
{
public:
  explicit Init_ExitProductionTestMode_Response_error_details(::common_interface::srv::ExitProductionTestMode_Response & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::ExitProductionTestMode_Response error_details(::common_interface::srv::ExitProductionTestMode_Response::_error_details_type arg)
  {
    msg_.error_details = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::ExitProductionTestMode_Response msg_;
};

class Init_ExitProductionTestMode_Response_error_codes
{
public:
  explicit Init_ExitProductionTestMode_Response_error_codes(::common_interface::srv::ExitProductionTestMode_Response & msg)
  : msg_(msg)
  {}
  Init_ExitProductionTestMode_Response_error_details error_codes(::common_interface::srv::ExitProductionTestMode_Response::_error_codes_type arg)
  {
    msg_.error_codes = std::move(arg);
    return Init_ExitProductionTestMode_Response_error_details(msg_);
  }

private:
  ::common_interface::srv::ExitProductionTestMode_Response msg_;
};

class Init_ExitProductionTestMode_Response_error_code
{
public:
  Init_ExitProductionTestMode_Response_error_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExitProductionTestMode_Response_error_codes error_code(::common_interface::srv::ExitProductionTestMode_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ExitProductionTestMode_Response_error_codes(msg_);
  }

private:
  ::common_interface::srv::ExitProductionTestMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::ExitProductionTestMode_Response>()
{
  return common_interface::srv::builder::Init_ExitProductionTestMode_Response_error_code();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__EXIT_PRODUCTION_TEST_MODE__BUILDER_HPP_
