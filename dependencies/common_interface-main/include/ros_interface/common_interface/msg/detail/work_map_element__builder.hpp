// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_interface:msg/WorkMapElement.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__BUILDER_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_interface/msg/detail/work_map_element__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_interface
{

namespace msg
{

namespace builder
{

class Init_WorkMapElement_polygon
{
public:
  explicit Init_WorkMapElement_polygon(::common_interface::msg::WorkMapElement & msg)
  : msg_(msg)
  {}
  ::common_interface::msg::WorkMapElement polygon(::common_interface::msg::WorkMapElement::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_interface::msg::WorkMapElement msg_;
};

class Init_WorkMapElement_contour_type
{
public:
  explicit Init_WorkMapElement_contour_type(::common_interface::msg::WorkMapElement & msg)
  : msg_(msg)
  {}
  Init_WorkMapElement_polygon contour_type(::common_interface::msg::WorkMapElement::_contour_type_type arg)
  {
    msg_.contour_type = std::move(arg);
    return Init_WorkMapElement_polygon(msg_);
  }

private:
  ::common_interface::msg::WorkMapElement msg_;
};

class Init_WorkMapElement_element_id
{
public:
  Init_WorkMapElement_element_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorkMapElement_contour_type element_id(::common_interface::msg::WorkMapElement::_element_id_type arg)
  {
    msg_.element_id = std::move(arg);
    return Init_WorkMapElement_contour_type(msg_);
  }

private:
  ::common_interface::msg::WorkMapElement msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_interface::msg::WorkMapElement>()
{
  return common_interface::msg::builder::Init_WorkMapElement_element_id();
}

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_ELEMENT__BUILDER_HPP_
