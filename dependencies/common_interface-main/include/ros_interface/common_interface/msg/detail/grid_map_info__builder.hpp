// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/GridMapInfo.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/grid_map_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_GridMapInfo_type
{
public:
  explicit Init_GridMapInfo_type(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::GridMapInfo type(::common_interface::msg::GridMapInfo::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_max_h_cm
{
public:
  explicit Init_GridMapInfo_max_h_cm(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  Init_GridMapInfo_type max_h_cm(::common_interface::msg::GridMapInfo::_max_h_cm_type arg)
  {
    msg_.max_h_cm = std::move(arg);
    return Init_GridMapInfo_type(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_min_h_cm
{
public:
  explicit Init_GridMapInfo_min_h_cm(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  Init_GridMapInfo_max_h_cm min_h_cm(::common_interface::msg::GridMapInfo::_min_h_cm_type arg)
  {
    msg_.min_h_cm = std::move(arg);
    return Init_GridMapInfo_max_h_cm(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_avg_h_cm
{
public:
  explicit Init_GridMapInfo_avg_h_cm(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  Init_GridMapInfo_min_h_cm avg_h_cm(::common_interface::msg::GridMapInfo::_avg_h_cm_type arg)
  {
    msg_.avg_h_cm = std::move(arg);
    return Init_GridMapInfo_min_h_cm(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_resolution_mm
{
public:
  explicit Init_GridMapInfo_resolution_mm(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  Init_GridMapInfo_avg_h_cm resolution_mm(::common_interface::msg::GridMapInfo::_resolution_mm_type arg)
  {
    msg_.resolution_mm = std::move(arg);
    return Init_GridMapInfo_avg_h_cm(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_cols
{
public:
  explicit Init_GridMapInfo_cols(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  Init_GridMapInfo_resolution_mm cols(::common_interface::msg::GridMapInfo::_cols_type arg)
  {
    msg_.cols = std::move(arg);
    return Init_GridMapInfo_resolution_mm(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_rows
{
public:
  explicit Init_GridMapInfo_rows(::common_interface::msg::GridMapInfo & msg)
  : msg_(msg)
  {}
  Init_GridMapInfo_cols rows(::common_interface::msg::GridMapInfo::_rows_type arg)
  {
    msg_.rows = std::move(arg);
    return Init_GridMapInfo_cols(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

class Init_GridMapInfo_header
{
public:
  Init_GridMapInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GridMapInfo_rows header(::common_interface::msg::GridMapInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GridMapInfo_rows(msg_);
  }

private:
  ::common_interface::msg::GridMapInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::GridMapInfo>()
{
  return common_interface::msg::builder::Init_GridMapInfo_header();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__GRID_MAP_INFO__BUILDER_HPP_
