// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:srv/RTKVlsam.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__BUILDER_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/srv/detail/rtk_vlsam__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_RTKVlsam_Request_map2enu_qw
{
public:
  explicit Init_RTKVlsam_Request_map2enu_qw(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::RTKVlsam_Request map2enu_qw(::common_interface::srv::RTKVlsam_Request::_map2enu_qw_type arg)
  {
    msg_.map2enu_qw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_map2enu_qz
{
public:
  explicit Init_RTKVlsam_Request_map2enu_qz(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_qw map2enu_qz(::common_interface::srv::RTKVlsam_Request::_map2enu_qz_type arg)
  {
    msg_.map2enu_qz = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_qw(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_map2enu_qy
{
public:
  explicit Init_RTKVlsam_Request_map2enu_qy(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_qz map2enu_qy(::common_interface::srv::RTKVlsam_Request::_map2enu_qy_type arg)
  {
    msg_.map2enu_qy = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_qz(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_map2enu_qx
{
public:
  explicit Init_RTKVlsam_Request_map2enu_qx(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_qy map2enu_qx(::common_interface::srv::RTKVlsam_Request::_map2enu_qx_type arg)
  {
    msg_.map2enu_qx = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_qy(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_map2enu_tz
{
public:
  explicit Init_RTKVlsam_Request_map2enu_tz(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_qx map2enu_tz(::common_interface::srv::RTKVlsam_Request::_map2enu_tz_type arg)
  {
    msg_.map2enu_tz = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_qx(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_map2enu_ty
{
public:
  explicit Init_RTKVlsam_Request_map2enu_ty(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_tz map2enu_ty(::common_interface::srv::RTKVlsam_Request::_map2enu_ty_type arg)
  {
    msg_.map2enu_ty = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_tz(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_map2enu_tx
{
public:
  explicit Init_RTKVlsam_Request_map2enu_tx(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_ty map2enu_tx(::common_interface::srv::RTKVlsam_Request::_map2enu_tx_type arg)
  {
    msg_.map2enu_tx = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_ty(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_origin_altitude
{
public:
  explicit Init_RTKVlsam_Request_origin_altitude(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_map2enu_tx origin_altitude(::common_interface::srv::RTKVlsam_Request::_origin_altitude_type arg)
  {
    msg_.origin_altitude = std::move(arg);
    return Init_RTKVlsam_Request_map2enu_tx(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_origin_longitude
{
public:
  explicit Init_RTKVlsam_Request_origin_longitude(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_origin_altitude origin_longitude(::common_interface::srv::RTKVlsam_Request::_origin_longitude_type arg)
  {
    msg_.origin_longitude = std::move(arg);
    return Init_RTKVlsam_Request_origin_altitude(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_origin_latitude
{
public:
  explicit Init_RTKVlsam_Request_origin_latitude(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_origin_longitude origin_latitude(::common_interface::srv::RTKVlsam_Request::_origin_latitude_type arg)
  {
    msg_.origin_latitude = std::move(arg);
    return Init_RTKVlsam_Request_origin_longitude(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_has_dock
{
public:
  explicit Init_RTKVlsam_Request_has_dock(::common_interface::srv::RTKVlsam_Request & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Request_origin_latitude has_dock(::common_interface::srv::RTKVlsam_Request::_has_dock_type arg)
  {
    msg_.has_dock = std::move(arg);
    return Init_RTKVlsam_Request_origin_latitude(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

class Init_RTKVlsam_Request_cmd
{
public:
  Init_RTKVlsam_Request_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RTKVlsam_Request_has_dock cmd(::common_interface::srv::RTKVlsam_Request::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_RTKVlsam_Request_has_dock(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::RTKVlsam_Request>()
{
  return common_interface::srv::builder::Init_RTKVlsam_Request_cmd();
}

}  // namespace common_interface


namespace common_interface
{

namespace srv
{

namespace builder
{

class Init_RTKVlsam_Response_message
{
public:
  explicit Init_RTKVlsam_Response_message(::common_interface::srv::RTKVlsam_Response & msg)
  : msg_(msg)
  {}
  ::common_interface::srv::RTKVlsam_Response message(::common_interface::srv::RTKVlsam_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Response msg_;
};

class Init_RTKVlsam_Response_error_code
{
public:
  explicit Init_RTKVlsam_Response_error_code(::common_interface::srv::RTKVlsam_Response & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Response_message error_code(::common_interface::srv::RTKVlsam_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_RTKVlsam_Response_message(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Response msg_;
};

class Init_RTKVlsam_Response_session_id
{
public:
  explicit Init_RTKVlsam_Response_session_id(::common_interface::srv::RTKVlsam_Response & msg)
  : msg_(msg)
  {}
  Init_RTKVlsam_Response_error_code session_id(::common_interface::srv::RTKVlsam_Response::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_RTKVlsam_Response_error_code(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Response msg_;
};

class Init_RTKVlsam_Response_result
{
public:
  Init_RTKVlsam_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RTKVlsam_Response_session_id result(::common_interface::srv::RTKVlsam_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_RTKVlsam_Response_session_id(msg_);
  }

private:
  ::common_interface::srv::RTKVlsam_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::srv::RTKVlsam_Response>()
{
  return common_interface::srv::builder::Init_RTKVlsam_Response_result();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__BUILDER_HPP_
