// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/WorkMapPoint.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_POINT__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/work_map_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_WorkMapPoint_y
{
public:
  explicit Init_WorkMapPoint_y(::common_interface::msg::WorkMapPoint & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::WorkMapPoint y(::common_interface::msg::WorkMapPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::WorkMapPoint msg_;
};

class Init_WorkMapPoint_x
{
public:
  Init_WorkMapPoint_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorkMapPoint_y x(::common_interface::msg::WorkMapPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_WorkMapPoint_y(msg_);
  }

private:
  ::common_interface::msg::WorkMapPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::WorkMapPoint>()
{
  return common_interface::msg::builder::Init_WorkMapPoint_x();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_POINT__BUILDER_HPP_
