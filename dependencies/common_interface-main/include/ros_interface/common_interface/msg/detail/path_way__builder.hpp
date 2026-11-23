// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/PathWay.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/path_way__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_PathWay_description
{
public:
  explicit Init_PathWay_description(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::PathWay description(::common_interface::msg::PathWay::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_pathway_width
{
public:
  explicit Init_PathWay_pathway_width(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  Init_PathWay_description pathway_width(::common_interface::msg::PathWay::_pathway_width_type arg)
  {
    msg_.pathway_width = std::move(arg);
    return Init_PathWay_description(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_polygon
{
public:
  explicit Init_PathWay_polygon(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  Init_PathWay_pathway_width polygon(::common_interface::msg::PathWay::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return Init_PathWay_pathway_width(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_to_zone_id
{
public:
  explicit Init_PathWay_to_zone_id(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  Init_PathWay_polygon to_zone_id(::common_interface::msg::PathWay::_to_zone_id_type arg)
  {
    msg_.to_zone_id = std::move(arg);
    return Init_PathWay_polygon(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_from_zone_id
{
public:
  explicit Init_PathWay_from_zone_id(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  Init_PathWay_to_zone_id from_zone_id(::common_interface::msg::PathWay::_from_zone_id_type arg)
  {
    msg_.from_zone_id = std::move(arg);
    return Init_PathWay_to_zone_id(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_parent_id
{
public:
  explicit Init_PathWay_parent_id(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  Init_PathWay_from_zone_id parent_id(::common_interface::msg::PathWay::_parent_id_type arg)
  {
    msg_.parent_id = std::move(arg);
    return Init_PathWay_from_zone_id(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_pathway_name
{
public:
  explicit Init_PathWay_pathway_name(::common_interface::msg::PathWay & msg)
  : msg_(msg)
  {}
  Init_PathWay_parent_id pathway_name(::common_interface::msg::PathWay::_pathway_name_type arg)
  {
    msg_.pathway_name = std::move(arg);
    return Init_PathWay_parent_id(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

class Init_PathWay_pathway_id
{
public:
  Init_PathWay_pathway_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathWay_pathway_name pathway_id(::common_interface::msg::PathWay::_pathway_id_type arg)
  {
    msg_.pathway_id = std::move(arg);
    return Init_PathWay_pathway_name(msg_);
  }

private:
  ::common_interface::msg::PathWay msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::PathWay>()
{
  return common_interface::msg::builder::Init_PathWay_pathway_id();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__PATH_WAY__BUILDER_HPP_
