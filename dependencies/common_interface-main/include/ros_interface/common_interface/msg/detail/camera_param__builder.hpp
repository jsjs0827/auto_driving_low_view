// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/CameraParam.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/camera_param__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_CameraParam_dsp_max_value
{
public:
  explicit Init_CameraParam_dsp_max_value(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::CameraParam dsp_max_value(::common_interface::msg::CameraParam::_dsp_max_value_type arg)
  {
    msg_.dsp_max_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_height
{
public:
  explicit Init_CameraParam_height(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_dsp_max_value height(::common_interface::msg::CameraParam::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_CameraParam_dsp_max_value(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_width
{
public:
  explicit Init_CameraParam_width(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_height width(::common_interface::msg::CameraParam::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_CameraParam_height(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_base_line
{
public:
  explicit Init_CameraParam_base_line(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_width base_line(::common_interface::msg::CameraParam::_base_line_type arg)
  {
    msg_.base_line = std::move(arg);
    return Init_CameraParam_width(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_bxf
{
public:
  explicit Init_CameraParam_bxf(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_base_line bxf(::common_interface::msg::CameraParam::_bxf_type arg)
  {
    msg_.bxf = std::move(arg);
    return Init_CameraParam_base_line(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_tangential_distortion_p
{
public:
  explicit Init_CameraParam_tangential_distortion_p(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_bxf tangential_distortion_p(::common_interface::msg::CameraParam::_tangential_distortion_p_type arg)
  {
    msg_.tangential_distortion_p = std::move(arg);
    return Init_CameraParam_bxf(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_radial_distortion_k
{
public:
  explicit Init_CameraParam_radial_distortion_k(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_tangential_distortion_p radial_distortion_k(::common_interface::msg::CameraParam::_radial_distortion_k_type arg)
  {
    msg_.radial_distortion_k = std::move(arg);
    return Init_CameraParam_tangential_distortion_p(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_cy
{
public:
  explicit Init_CameraParam_cy(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_radial_distortion_k cy(::common_interface::msg::CameraParam::_cy_type arg)
  {
    msg_.cy = std::move(arg);
    return Init_CameraParam_radial_distortion_k(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_cx
{
public:
  explicit Init_CameraParam_cx(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_cy cx(::common_interface::msg::CameraParam::_cx_type arg)
  {
    msg_.cx = std::move(arg);
    return Init_CameraParam_cy(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_fy
{
public:
  explicit Init_CameraParam_fy(::common_interface::msg::CameraParam & msg)
  : msg_(msg)
  {}
  Init_CameraParam_cx fy(::common_interface::msg::CameraParam::_fy_type arg)
  {
    msg_.fy = std::move(arg);
    return Init_CameraParam_cx(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

class Init_CameraParam_fx
{
public:
  Init_CameraParam_fx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraParam_fy fx(::common_interface::msg::CameraParam::_fx_type arg)
  {
    msg_.fx = std::move(arg);
    return Init_CameraParam_fy(msg_);
  }

private:
  ::common_interface::msg::CameraParam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::CameraParam>()
{
  return common_interface::msg::builder::Init_CameraParam_fx();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__BUILDER_HPP_
