// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/EnterProductionTestMode.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__ENTER_PRODUCTION_TEST_MODE__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__ENTER_PRODUCTION_TEST_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/enter_production_test_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_EnterProductionTestMode_Request_expected_version
{
public:
  Init_EnterProductionTestMode_Request_expected_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::common_interface::srv::EnterProductionTestMode_Request expected_version(::common_interface::srv::EnterProductionTestMode_Request::_expected_version_type arg)
  {
    msg_.expected_version = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::EnterProductionTestMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::EnterProductionTestMode_Request>()
{
  return common_interface::srv::builder::Init_EnterProductionTestMode_Request_expected_version();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_EnterProductionTestMode_Response_error_details
{
public:
  explicit Init_EnterProductionTestMode_Response_error_details(::common_interface::srv::EnterProductionTestMode_Response & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::EnterProductionTestMode_Response error_details(::common_interface::srv::EnterProductionTestMode_Response::_error_details_type arg)
  {
    msg_.error_details = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::EnterProductionTestMode_Response msg_;
};

class Init_EnterProductionTestMode_Response_error_codes
{
public:
  explicit Init_EnterProductionTestMode_Response_error_codes(::common_interface::srv::EnterProductionTestMode_Response & msg)
  : msg_(msg)
  {}
  Init_EnterProductionTestMode_Response_error_details error_codes(::common_interface::srv::EnterProductionTestMode_Response::_error_codes_type arg)
  {
    msg_.error_codes = std::move(arg);
    return Init_EnterProductionTestMode_Response_error_details(msg_);
  }

private:
  ::common_interface::srv::EnterProductionTestMode_Response msg_;
};

class Init_EnterProductionTestMode_Response_error_code
{
public:
  Init_EnterProductionTestMode_Response_error_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EnterProductionTestMode_Response_error_codes error_code(::common_interface::srv::EnterProductionTestMode_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_EnterProductionTestMode_Response_error_codes(msg_);
  }

private:
  ::common_interface::srv::EnterProductionTestMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::EnterProductionTestMode_Response>()
{
  return common_interface::srv::builder::Init_EnterProductionTestMode_Response_error_code();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__ENTER_PRODUCTION_TEST_MODE__BUILDER_HPP_
