// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/WorkMapPoint.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_POINT__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__WorkMapPoint __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__WorkMapPoint __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorkMapPoint_
{
  using Type = WorkMapPoint_<ContainerAllocator>;

  explicit WorkMapPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit WorkMapPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::WorkMapPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::WorkMapPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMapPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::WorkMapPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__WorkMapPoint
    std::shared_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__WorkMapPoint
    std::shared_ptr<common_interface::msg::WorkMapPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorkMapPoint_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorkMapPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorkMapPoint_

// alias to use template instance with default allocator
using WorkMapPoint =
  common_interface::msg::WorkMapPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__WORK_MAP_POINT__STRUCT_HPP_
