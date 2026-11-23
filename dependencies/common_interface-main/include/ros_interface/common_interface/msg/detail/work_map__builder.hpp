// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/WorkMap.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/work_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_WorkMap_pathways
{
public:
  explicit Init_WorkMap_pathways(::common_interface::msg::WorkMap & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::WorkMap pathways(::common_interface::msg::WorkMap::_pathways_type arg)
  {
    msg_.pathways = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::WorkMap msg_;
};

class Init_WorkMap_zones
{
public:
  explicit Init_WorkMap_zones(::common_interface::msg::WorkMap & msg)
  : msg_(msg)
  {}
  Init_WorkMap_pathways zones(::common_interface::msg::WorkMap::_zones_type arg)
  {
    msg_.zones = std::move(arg);
    return Init_WorkMap_pathways(msg_);
  }

private:
  ::common_interface::msg::WorkMap msg_;
};

class Init_WorkMap_active
{
public:
  explicit Init_WorkMap_active(::common_interface::msg::WorkMap & msg)
  : msg_(msg)
  {}
  Init_WorkMap_zones active(::common_interface::msg::WorkMap::_active_type arg)
  {
    msg_.active = std::move(arg);
    return Init_WorkMap_zones(msg_);
  }

private:
  ::common_interface::msg::WorkMap msg_;
};

class Init_WorkMap_frame_id
{
public:
  explicit Init_WorkMap_frame_id(::common_interface::msg::WorkMap & msg)
  : msg_(msg)
  {}
  Init_WorkMap_active frame_id(::common_interface::msg::WorkMap::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_WorkMap_active(msg_);
  }

private:
  ::common_interface::msg::WorkMap msg_;
};

class Init_WorkMap_map_name
{
public:
  explicit Init_WorkMap_map_name(::common_interface::msg::WorkMap & msg)
  : msg_(msg)
  {}
  Init_WorkMap_frame_id map_name(::common_interface::msg::WorkMap::_map_name_type arg)
  {
    msg_.map_name = std::move(arg);
    return Init_WorkMap_frame_id(msg_);
  }

private:
  ::common_interface::msg::WorkMap msg_;
};

class Init_WorkMap_map_id
{
public:
  Init_WorkMap_map_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorkMap_map_name map_id(::common_interface::msg::WorkMap::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return Init_WorkMap_map_name(msg_);
  }

private:
  ::common_interface::msg::WorkMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::WorkMap>()
{
  return common_interface::msg::builder::Init_WorkMap_map_id();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP__BUILDER_HPP_
