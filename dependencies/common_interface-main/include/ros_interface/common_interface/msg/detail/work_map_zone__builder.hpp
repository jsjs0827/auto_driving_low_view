// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/WorkMapZone.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/work_map_zone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_WorkMapZone_elements
{
public:
  explicit Init_WorkMapZone_elements(::common_interface::msg::WorkMapZone & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::WorkMapZone elements(::common_interface::msg::WorkMapZone::_elements_type arg)
  {
    msg_.elements = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::WorkMapZone msg_;
};

class Init_WorkMapZone_parent_id
{
public:
  explicit Init_WorkMapZone_parent_id(::common_interface::msg::WorkMapZone & msg)
  : msg_(msg)
  {}
  Init_WorkMapZone_elements parent_id(::common_interface::msg::WorkMapZone::_parent_id_type arg)
  {
    msg_.parent_id = std::move(arg);
    return Init_WorkMapZone_elements(msg_);
  }

private:
  ::common_interface::msg::WorkMapZone msg_;
};

class Init_WorkMapZone_zone_name
{
public:
  explicit Init_WorkMapZone_zone_name(::common_interface::msg::WorkMapZone & msg)
  : msg_(msg)
  {}
  Init_WorkMapZone_parent_id zone_name(::common_interface::msg::WorkMapZone::_zone_name_type arg)
  {
    msg_.zone_name = std::move(arg);
    return Init_WorkMapZone_parent_id(msg_);
  }

private:
  ::common_interface::msg::WorkMapZone msg_;
};

class Init_WorkMapZone_zone_id
{
public:
  Init_WorkMapZone_zone_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorkMapZone_zone_name zone_id(::common_interface::msg::WorkMapZone::_zone_id_type arg)
  {
    msg_.zone_id = std::move(arg);
    return Init_WorkMapZone_zone_name(msg_);
  }

private:
  ::common_interface::msg::WorkMapZone msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::WorkMapZone>()
{
  return common_interface::msg::builder::Init_WorkMapZone_zone_id();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ZONE__BUILDER_HPP_
